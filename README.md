#include "mbed.h"
#include "LKMotor.h"

CAN can(PA_11, PA_12, 1000000);       // CAN設定
LKMotor motor(can, 4);                // モーター1台管理
UnbufferedSerial pc(USBTX, USBRX, 115200);  
LKMState state;//構造体

int32_t speeds[4] = {0};
char key;
int main() {
    CANMessage msg;

    while (true) {
        if (pc.readable()) {
            pc.read(&key, 1);
            switch (key) {
                case 'w': speeds[0] = 60;   break;   // 正転
                case 'q': speeds[0] = 120;  break;  
                case 's': speeds[0] = -60;  break;  // 逆転
                case 'x': speeds[0] = 0;    break;           // 停止
            }
        }
        // CAN受信処理（常時）
        if (can.read(msg)) {
            motor.get_msg(msg);
        }
        
        // モーターに速度制御コマンドを送信
        motor.spd_controlAll(speeds);   //配列にしたやつ全部送信
        //motor.spd_control(0,speeds[0]); //idが裏のディップスイッチが0001のモーターに送信

        motor.requestAll(0); // モーターの状態を要求

        if(motor.get_state(0, state)){// 受けとった状態を取得
            printf("temp: %d  spd: %d  enc: %d ",
                state.temp, state.spd, state.deg);
            printf("voltage: %d ",state.volt);
            printf("totaldeg: %d deg\n", (int)state.total_deg);
            printf("\r\n");
        }
    }
}
