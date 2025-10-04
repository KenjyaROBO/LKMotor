    #include "mbed.h"
    #include "LKMotor.h"
    
    CAN can(PD_0, PD_1, 1000000);
    LKMotor motor(can, 1);
    UnbufferedSerial pc(USBTX, USBRX, 115200);
    LKMState state;
    
    int32_t speeds[1] = {0};
    char key;
    
    int main() {
        CANMessage msg;
        int temp = 0, spd = 0, deg = 0;
        uint16_t volt = 0;
    
    
        while (true) {
            if (pc.readable()) {
                pc.read(&key, 1);
                switch (key) {
                    case 'w': speeds[0] = 60;   break;
                    case 's': speeds[0] = -90;  break;
                    case 'x': speeds[0] = 0;    break;
                }
            }
            
            if (can.read(msg)) motor.get_msg(msg);
            
            motor.spd_controlAll(speeds);
    
            motor.request(0); 
    
            if(motor.get_state(0, state)){
                temp = state.temp;
                spd = state.spd;
                deg = state.deg;
                volt = state.volt;
            }
    
            printf("temp: %d  spd: %d  enc: %d  volt: %d\r\n", temp, spd, deg, volt);
            ThisThread::sleep_for(3ms);
        }
    }

