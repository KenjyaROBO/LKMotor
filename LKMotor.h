#ifndef LKMOTOR_H
#define LKMOTOR_H

#include "mbed.h"
#include <cstdint>

struct LKMState {
    uint8_t temp;
    uint16_t volt;
    int16_t trq_cur;
    int16_t spd;
    int16_t deg;
    double total_deg;
    bool updated9C;
    bool updated9A;
    bool updated92;
};

class LKMotor {
public:
    LKMotor(CAN &can, uint8_t motor_count);

    void motorOn();
    void motorOff();
    void stopMotor();

    void trq_control(int motor_index, int16_t trqControl);
    void spd_control(int motor_index, int32_t speedControl);

    void angle_control(int motor_index, int32_t angleControl);
    void set_angle_control(int motor_index, int32_t angleControl, uint16_t maxSpeed);
    void abs_angle_control(int motor_index,  uint32_t angleControl,uint8_t spinDirection);
    void set_abs_angle_control(int motor_index, uint32_t angleControl,uint8_t spinDirection, uint16_t maxSpeed);
    void inc_angle_control(int motor_index, int32_t angleIncrement);
    void set_inc_angle_control(int motor_index, int32_t angleIncrement, uint16_t maxSpeed);

    void trq_controlAll(int16_t* torques);
    void spd_controlAll(int32_t* speeds);
    void angle_controlAll(int32_t* angles);
    void set_angle_controlAll(int32_t* angles,uint16_t* speeds);
    void abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections);
    void set_abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections, uint16_t* maxSpeeds);
    void inc_angle_controlAll(int32_t* angleIncrements);
    void set_inc_angle_controlAll(int32_t* angleIncrements, uint16_t* maxSpeeds);

    void get_msg(const CANMessage &msg);
    bool get_state(uint8_t index, LKMState &status);

    void request_state1(uint8_t index); // 0x9A
    void request_state2(uint8_t index); // 0x9C
    void request_encoder(uint8_t index); // 0x92

    void requestAll(uint8_t index);

private:
    CAN &_can;
    uint8_t _motorCount;
    short count = 0;
    LKMState _status[8];

    uint16_t getCanID(uint8_t motorID) const;
    void sendCommand(uint8_t motorID, uint8_t cmd, const uint8_t data[8]);
};

#endif