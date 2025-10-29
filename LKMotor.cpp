#include "LKMotor.h"
#include <cstdint>

LKMotor::LKMotor(CAN &can, uint8_t motor_count)
    : _can(can), _motorCount(motor_count) {
    for (int i = 0; i < 8; i++) {
        _status[i] = {0, 0, 0, 0, 0, 0, false, false, false, false};//構造体初期化
    }
    motorOn(); //最初にモーターオン指令(なくてもいける？)
}

uint16_t LKMotor::getCanID(uint8_t motorID) const { //モーターのid割り振り
    return 0x140 + motorID;
}

void LKMotor::sendCommand(uint8_t motorID, uint8_t cmd, const uint8_t data[8]) { //モーターへの指示送信関数
    uint8_t frame[8];
    frame[0] = cmd; //送信データ0番目にコマンドid
    for (int i = 1; i < 8; i++) {
        frame[i] = data[i]; //送信データの1~7番目に送信データの情報を書き込む(コマンドによって配置異なる)
    }
    _can.write(CANMessage(getCanID(motorID), (char*)frame, 8));//CANデータ送信
    if(cmd == 0xA2) A2_flag = false; //0xA2(速度制御)だけ速度制御しながらrpmや角度の情報得られる
    else A2_flag = true; //けどA2_flagでうまくデータのやりとりしないと電圧とか回転総角度のデータを受け取れなくなる
}

void LKMotor::motorOn() { //モーターオン
    uint8_t data[8] = {0x88,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x88, data);
}

void LKMotor::motorOff() {//モーターオフ(多分フィードバックしなくなる？)
    uint8_t data[8] = {0x80,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x80, data);
}

void LKMotor::stopMotor() {//モーター止める
    uint8_t data[8] = {0x81,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x81, data);
}

void LKMotor::trq_control(int motor_index, int16_t torqueControl) {//トルク制御(個別送信)
    if(torqueControl > 2048)  torqueControl =  2048; //トルクの最大値2048
    if(torqueControl < -2048) torqueControl = -2048;
    uint8_t data[8] = {0xA1, 0, 0, 0,
        (uint8_t)(torqueControl & 0xFF),//トルクの値の上位ビット
        (uint8_t)((torqueControl >> 8) & 0xFF),//下位ビット
        0, 0};//配列0番目にコマンド(0xA1),4と5番目にトルクの値を書き込む
    if(!state_flag) sendCommand(motor_index + 1, 0xA1, data);//モーター状態確認コマンド(0x9C,9A等)を送信したいときは送信しない
}

void LKMotor::trq_controlAll(int16_t* torques){//トルク制御(一斉送信)
    for (int i = 0; i < _motorCount; i++) trq_control(i, torques[i]);
}

//速度制御　これだけmotor.request_state1をしなくてもrpmと絶対角とトルク電流と温度のデータは受けとれる
void LKMotor::spd_control(int motor_index, int32_t speedControl) {
    if(speedControl > 1900)  speedControl =  1900; //max 1900rpm
    if(speedControl < -1900) speedControl = -1900;
    speedControl*=(int32_t)600; //1[rpm] = 0.01[dps] * 100 * 360[°] / 60[s]
    uint8_t* s = (uint8_t*)&speedControl;
    uint8_t data[8] = {0xA2, 0, 0, 0, s[0], s[1], s[2], s[3]};
    if(!state_flag && A2_flag) sendCommand(motor_index + 1, 0xA2, data);
}

void LKMotor::spd_controlAll(int32_t* speeds) {
    for (int i = 0; i < _motorCount; i++) spd_control(i, speeds[i]);
}

//角度制御(爆速) 負荷時はPIDがオーバーシュートするから非推奨
void LKMotor::angle_control(int motor_index, int32_t angleControl) {
    if(angleControl > 21474836)  angleControl =  21474836; //max
    if(angleControl < -21474836) angleControl = -21474836; //min
    angleControl *= (int32_t)100;
    uint8_t data[8] = {0};
    data[0] = 0xA3;
    uint8_t* p = (uint8_t*)&angleControl;
    data[4] = p[0]; data[5] = p[1]; data[6] = p[2]; data[7] = p[3];
    if(!state_flag) sendCommand(motor_index + 1, 0xA3, data);
}

void LKMotor::angle_controlAll(int32_t* angles) {
    for (int i = 0; i < _motorCount; i++) angle_control(i, angles[i]);
}

//角度制御の速度決めれるバージョン　角度制御したいなら基本こっち
void LKMotor::set_angle_control(int motor_index, int32_t angleControl, uint16_t maxSpeed) {
    if(angleControl > 21474836)  angleControl =  21474836;
    if(angleControl < -21474836) angleControl = -21474836;
    if(maxSpeed > 1500)  maxSpeed =  1500;
    angleControl *= (int32_t)100;
    maxSpeed *= (uint16_t)6; //[rpm] = 1[dps] * 360[°] / 60[s]
    uint8_t* p = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA4, 0, 
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        p[0], p[1], p[2], p[3]};
    if(!state_flag) sendCommand(motor_index + 1, 0xA4, data);
}

void LKMotor::set_angle_controlAll(int32_t* angles,uint16_t* speeds){
    for (int i = 0; i < _motorCount; i++) set_angle_control(i, angles[i],speeds[i]);
}

//角度制御絶対角(0°~360°)バージョン(爆速) spinDirectionは穴が0が正転,1が反転
void LKMotor::abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection) {
    angleControl *= (uint32_t)100;
    uint8_t* p = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA5, spinDirection, 0, 0, p[0], p[1], p[2], p[3]};
    if(!state_flag) sendCommand(motor_index + 1, 0xA5, data);
}

void LKMotor::abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections) {
    for (int i = 0; i < _motorCount; i++) abs_angle_control(i, angles[i],spinDirections[i]);
}

//角度制御絶対角&回転速度決めれるバージョン
void LKMotor::set_abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed) {
    if(maxSpeed > 1500)  maxSpeed =  1500;
    angleControl *= (uint32_t)100;
    maxSpeed *= (uint16_t)6;//[rpm] = 1[dps] * 360[°] / 60[s]
    uint8_t* a = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA6, spinDirection,
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        a[0], a[1], a[2], a[3]};
    if(!state_flag) sendCommand(motor_index + 1, 0xA6, data);
}

void LKMotor::set_abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections, uint16_t* maxSpeeds) {
    for (int i = 0; i < _motorCount; i++) set_abs_angle_control(i, angles[i],spinDirections[i],maxSpeeds[i]);
}

//角度制御の現在角からさらに角度制御するバージョン
void LKMotor::inc_angle_control(int motor_index, int32_t angleIncrement) {
    angleIncrement *= (int32_t)100;
    uint8_t* p = (uint8_t*)&angleIncrement;
    uint8_t data[8] = {0xA7, 0, 0, 0, p[0], p[1], p[2], p[3]};
    if(!state_flag) sendCommand(motor_index + 1, 0xA7, data);
}

void LKMotor::inc_angle_controlAll(int32_t* angleIncrements) {
    for (int i = 0; i < _motorCount; i++) inc_angle_control(i, angleIncrements[i]);
}

//速度制御付けたバージョン
void LKMotor::set_inc_angle_control(int motor_index, int32_t angleIncrement, uint16_t maxSpeed) {
    if(maxSpeed > 1900)  maxSpeed =  1900;
    angleIncrement *= (int32_t)100;
    maxSpeed *= (uint16_t)6;
    uint8_t* a = (uint8_t*)&angleIncrement;
    uint8_t data[8] = {0xA8, 0,
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        a[0], a[1], a[2], a[3]};
    if(!state_flag) sendCommand(motor_index + 1, 0xA8, data);
}

void LKMotor::set_inc_angle_controlAll(int32_t* angleIncrements, uint16_t* maxSpeeds) {
    for (int i = 0; i < _motorCount; i++) set_inc_angle_control(i, angleIncrements[i],maxSpeeds[i]);
}

//CANデータ受け取り
void LKMotor::get_msg(const CANMessage &msg) {
    if(msg.id >= 0x140  && msg.id <= 0x140 + _motorCount){
        for (int i = 0; i < _motorCount; i++) {
            if (msg.id == 0x140 + (i + 1)) {
                if (msg.data[0] == 0xA2 || msg.data[0] == 0x9C) {
                    _status[i].temp = msg.data[1];
                    _status[i].trq_cur = (msg.data[2] | (msg.data[3] << 8));
                    _status[i].spd = (static_cast<int16_t>(msg.data[5] << 8) | msg.data[4])*60/360;
                    _status[i].deg = (msg.data[6] | msg.data[7] << 8) *180 / 32766;
                    if(msg.data[0] == 0x9C) _status[i].updated9C = true; state_flag = false; A2_flag = true;
                    if(msg.data[0] == 0xA2) _status[i].updatedA2 = true; A2_flag = false;
                }
                else if (msg.data[0] == 0x9A) {
                    _status[i].temp = msg.data[1];
                    _status[i].volt = (msg.data[2] | (msg.data[3] << 8));
                    _status[i].updated9A = true;
                    state_flag = false;
                    A2_flag = true;
                }
                else if (msg.data[0] == 0x92) {
                    int64_t angle = 0;
                    for (int b = 0; b < 7; ++b) {
                        angle |= ((int64_t)msg.data[1 + b]) << (8 * b);
                    }

                    if (angle & (1LL << 55)) {
                        angle |= 0xFF00000000000000LL;
                    }
                    _status[i].total_deg = angle * 0.01; 
                    _status[i].updated92 = true;
                    state_flag = false;
                    A2_flag = true;
                }
            }
        }
    }
}

//モーター状態受け取り
bool LKMotor::get_state(uint8_t index, LKMState &status) {
    if (index >= _motorCount) return false;
    if (_status[index].updated9C || _status[index].updated9A || _status[index].updated92 || _status[index].updatedA2) {
        status = _status[index];
        _status[index].updated9C = false;
        _status[index].updated9A = false;
        _status[index].updated92 = false;
        _status[index].updatedA2 = false;
        return true;
    }
    return false;
}

//モーター状態要求(温度、トルク電流、回転速度、絶対角)
void LKMotor::request_state1(uint8_t index) {
    uint8_t data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x9A, data);
    state_flag = true;
}

void LKMotor::request_state1All(){
    for(int i = 0; i < _motorCount; i++) request_state1(i);
}
//モーター状態要求(温度、電圧)
void LKMotor::request_state2(uint8_t index) {
    uint8_t data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x9C, data);
    state_flag = true;
}

void LKMotor::request_state2All(){
    for(int i = 0; i < _motorCount; i++) request_state2(i);
}
//モーター状態要求(相対角)
void LKMotor::request_encoder(uint8_t index) {
    uint8_t data[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x92, data);
    state_flag = true;
}

void LKMotor::request_encoderAll() {
    for(int i = 0; i < _motorCount; i++) request_encoder(i);
}

//モーター状態要求(全部)
void LKMotor::request(uint8_t index) {
    
    if(count[index] == 0){
        request_state1(index);
        if(_status[index].updated9A) count[index]++;
    }
    else if(count[index] == 1){
        if(A2_flag) count[index]++;
        if(!A2_flag){
            request_state2(index);
            if(_status[index].updated9C) count[index]++;
        }
    }
    else if(count[index] == 2){
        request_encoder(index);
        if(_status[index].updated92) count[index] = 0;
    }
}

void LKMotor::requestAll(){
    for(int i = 0; i < _motorCount; i++) request(i);
}