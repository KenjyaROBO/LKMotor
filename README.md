    #include "mbed.h"
    #include "LKMotor.h"
    #include <cstdio>
    
    CAN can(PD_0, PD_1, 1000000);
    // CAN can(PB_8,PB_9, 1000000);
    LKMotor motor(can, 1);
    UnbufferedSerial pc(USBTX, USBRX, 115200);
    LKMState state;
    
    int32_t speeds[1] = {0};
    int32_t angle[1] = {0};
    char key;
    short flag = 0;
    
    int main() {
        CANMessage msg;
        int temp = 0, spd = 0, deg = 0;
        int ang = 0;
        uint16_t volt = 0;
    
        while (true) {
            if (pc.readable()) {
                pc.read(&key, 1);
                switch (key) {
                    case 'w': speeds[0] = 60; flag = 0; break;
                    case 'x': speeds[0] = 0;  flag = 0; break;
                    case 'q': angle[0] =  90; flag = 1; break;
                    case 'e': angle[0] = -90; flag = 1; break;
                }
            }
            
            if (can.read(msg)) motor.get_msg(msg);
            
            if(flag == 0)motor.spd_controlAll(speeds);
            if(flag == 1)motor.set_angle_control(0,angle[0],60);
    
            motor.request(0); 
    
            if(motor.get_state(0, state)){
                temp = state.temp;
                spd = state.spd;
                deg = state.deg;
                volt = state.volt;
                ang = state.total_deg;
            }
    
            //printf("temp: %d  spd: %d  enc: %d  volt: %d\r\n", temp, spd, deg, volt);
            printf(">now_rpm:%d",spd);
            printf(">angle:%d",ang);
            printf("\r\n");
            ThisThread::sleep_for(3ms);
        }
    }
