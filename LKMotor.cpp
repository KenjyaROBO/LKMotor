#include "LKMotor.h"
#include <cstdint>

LKMotor::LKMotor(CAN &can, uint8_t motor_count)
    : _can(can), _motorCount(motor_count) {
    for (int i = 0; i < 8; i++) {
        _status[i] = {0, 0, 0, 0, 0, 0, false, false, false};
    }
    motorOn();
}

uint16_t LKMotor::getCanID(uint8_t motorID) const {
    return 0x140 + motorID;
}

void LKMotor::sendCommand(uint8_t motorID, uint8_t cmd, const uint8_t data[8]) {
    uint8_t frame[8];
    frame[0] = cmd;
    for (int i = 1; i < 8; i++) {
        frame[i] = data[i];
    }
    _can.write(CANMessage(getCanID(motorID), (char*)frame, 8));
}

void LKMotor::motorOn() {
    uint8_t data[8] = {0x88,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x88, data);
}

void LKMotor::motorOff() {
    uint8_t data[8] = {0x80,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x80, data);
}

void LKMotor::stopMotor() {
    uint8_t data[8] = {0x81,0,0,0,0,0,0,0};
    for (int i = 1; i <= _motorCount; i++) sendCommand(i, 0x81, data);
}

void LKMotor::trq_control(int motor_index, int16_t torqueControl) {
    uint8_t data[8] = {0xA1, 0, 0, 0,
        (uint8_t)(torqueControl & 0xFF),
        (uint8_t)((torqueControl >> 8) & 0xFF),
        0, 0};
    sendCommand(motor_index + 1, 0xA1, data);
}

void LKMotor::trq_controlAll(int16_t* torques){
    for (int i = 0; i < _motorCount; i++) {
        spd_control(i, torques[i]);
    }
}

void LKMotor::spd_control(int motor_index, int32_t speedControl) {
    speedControl*=(int32_t)600;
    uint8_t* s = (uint8_t*)&speedControl;
    uint8_t data[8] = {0xA2, 0, 0, 0, s[0], s[1], s[2], s[3]};
    sendCommand(motor_index + 1, 0xA2, data);
}

void LKMotor::spd_controlAll(int32_t* speeds) {
    for (int i = 0; i < _motorCount; i++) {
        spd_control(i, speeds[i]);
    }
}

void LKMotor::angle_control(int motor_index, int32_t angleControl) {
    angleControl *= (int32_t)100;
    uint8_t data[8] = {0};
    data[0] = 0xA3;
    uint8_t* p = (uint8_t*)&angleControl;
    data[4] = p[0]; data[5] = p[1]; data[6] = p[2]; data[7] = p[3];
    sendCommand(motor_index + 1, 0xA3, data);
}

void LKMotor::angle_controlAll(int32_t* angles) {
    for (int i = 0; i < _motorCount; i++) {
        spd_control(i, angles[i]);
    }
}

void LKMotor::set_angle_control(int motor_index, int32_t angleControl, uint16_t maxSpeed) {
    angleControl *= (int32_t)100;
    maxSpeed *= (uint16_t)6;
    uint8_t* p = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA4, 0, 
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        p[0], p[1], p[2], p[3]};
    sendCommand(motor_index + 1, 0xA4, data);
}

void LKMotor::set_angle_controlAll(int32_t* angles,uint16_t* speeds){
    for (int i = 0; i < _motorCount; i++) {
        set_angle_control(i, angles[i],speeds[i]);
    }
}

void LKMotor::abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection) {
    angleControl *= (uint32_t)100;
    uint8_t* p = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA5, spinDirection, 0, 0, p[0], p[1], p[2], p[3]};
    sendCommand(motor_index + 1, 0xA5, data);
}

void LKMotor::abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections) {
    for (int i = 0; i < _motorCount; i++){
        abs_angle_control(i, angles[i],spinDirections[i]);
    }
}

void LKMotor::set_abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed) {
    angleControl *= (uint32_t)100;
    maxSpeed *= (uint16_t)6;
    uint8_t* a = (uint8_t*)&angleControl;
    uint8_t data[8] = {0xA6, spinDirection,
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        a[0], a[1], a[2], a[3]};
    sendCommand(motor_index + 1, 0xA6, data);
}

void LKMotor::set_abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections, uint16_t* maxSpeeds) {
    for (int i = 0; i < _motorCount; i++){
        set_abs_angle_control(i, angles[i],spinDirections[i],maxSpeeds[i]);
    }
}

void LKMotor::inc_angle_control(int motor_index, int32_t angleIncrement) {
    angleIncrement *= (int32_t)100;
    uint8_t* p = (uint8_t*)&angleIncrement;
    uint8_t data[8] = {0xA7, 0, 0, 0, p[0], p[1], p[2], p[3]};
    sendCommand(motor_index + 1, 0xA7, data);
}

void LKMotor::inc_angle_controlAll(int32_t* angleIncrements) {
    for (int i = 0; i < _motorCount; i++){
        inc_angle_control(i, angleIncrements[i]);
    }
}

void LKMotor::set_inc_angle_control(int motor_index, int32_t angleIncrement, uint16_t maxSpeed) {
    angleIncrement *= (int32_t)100;
    maxSpeed *= (uint16_t)6;
    uint8_t* a = (uint8_t*)&angleIncrement;
    uint8_t data[8] = {0xA8, 0,
        (uint8_t)(maxSpeed & 0xFF), (uint8_t)(maxSpeed >> 8),
        a[0], a[1], a[2], a[3]};
    sendCommand(motor_index + 1, 0xA8, data);
}

void LKMotor::set_inc_angle_controlAll(int32_t* angleIncrements, uint16_t* maxSpeeds) {
    for (int i = 0; i < _motorCount; i++){
        set_inc_angle_control(i, angleIncrements[i],maxSpeeds[i]);
    }
}

void LKMotor::get_msg(const CANMessage &msg) {
    for (int i = 0; i < _motorCount; i++) {
        if (msg.id == 0x140 + (i + 1)) {
            if (msg.data[0] == 0x9C) {
                _status[i].temp = msg.data[1];
                _status[i].trq_cur = (msg.data[2] | (msg.data[3] << 8));
                _status[i].spd = (int16_t)((msg.data[4] << 8) | msg.data[5])/435; //なんか知らんけど435ぐらいでちょうどよくなる
                _status[i].deg = (msg.data[6] | msg.data[7] << 8) *180 / 32766;
                _status[i].updated9C = true;
            }
            else if (msg.data[0] == 0x9A) {
                _status[i].temp = msg.data[1];
                _status[i].volt = (msg.data[2] | (msg.data[3] << 8));
                _status[i].updated9A = true;
            }
            else if (msg.data[0] == 0x92) {
                int64_t angle = 0;
                for (int b = 0; b < 7; ++b) {
                    angle |= ((int64_t)msg.data[1 + b]) << (8 * b);
                }
                // 符号拡張（7バイト → int64_t）
                if (angle & (1LL << 55)) {
                    angle |= 0xFF00000000000000LL;
                }
                _status[i].total_deg = angle * 0.01; // 0.01度単位 → 度数法
                _status[i].updated92 = true;
            }
        }
    }
}

bool LKMotor::get_state(uint8_t index, LKMState &status) {
    if (index >= _motorCount) return false;
    if (_status[index].updated9C || _status[index].updated9A || _status[index].updated92) {
        status = _status[index];
        _status[index].updated9C = false;
        _status[index].updated9A = false;
        _status[index].updated92 = false;
        return true;
    }
    return false;
}


void LKMotor::request_state1(uint8_t index) {
    uint8_t data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x9A, data);
}

void LKMotor::request_state1All(){
    for(int i = 0; i < _motorCount; i++){
        request_state1(i);
    }
}

void LKMotor::request_state2(uint8_t index) {
    uint8_t data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x9C, data);
}

void LKMotor::request_state2All(){
    for(int i = 0; i < _motorCount; i++){
        request_state2(i);
    }
}

void LKMotor::request_encoder(uint8_t index) {
    uint8_t data[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
    sendCommand(index + 1, 0x92, data);
}

void LKMotor::request_encoderAll() {
    for(int i = 0; i < _motorCount; i++){
        request_encoder(i);
    }
}

void LKMotor::request(uint8_t index) {
    if(count==0){request_state1(index); count++;}
    else if(count==1){request_state2(index); count++;}
    else if(count==2){request_encoder(index); count=0;}
}

void LKMotor::requestAll(){
    for(int i = 0; i < _motorCount; i++){
        request(i);
    }
}
/*
    /\_____/\
    \ l o l /
    /   さ  \
   /|   く  |\
   \|   ま  |/
    |_______|
    | \___/ |
     \/   \/

*/