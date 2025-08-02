#ifndef LKMOTOR_H
#define LKMOTOR_H

#include "mbed.h"
#include <cstdint>

struct MotorState {
    uint8_t temp;
    uint16_t volt;
    int16_t trq_cur;
    int16_t spd;
    int16_t deg;
    double abs_deg;
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

    void trq_control(int motor_index, int16_t iqControl);
    void spd_control(int motor_index, int32_t speedControl);

    void angle_control(int motor_index, int32_t angleControl);
    void set_angle_control(int motor_index, int32_t angleControl, uint16_t maxSpeed);
    void abs_angle_control(int motor_index, uint8_t spinDirection, uint32_t angleControl);
    void set_abs_angle_control(int motor_index, uint8_t spinDirection, uint16_t maxSpeed, uint32_t angleControl);
    void inc_angle_control(int motor_index, int32_t angleIncrement);
    void set_inc_angle_control(int motor_index, int32_t angleIncrement, uint16_t maxSpeed);

    void spd_controlAll(int32_t* speeds);

    void get_CANdata(const CANMessage &msg);
    bool get_state(uint8_t index, MotorState &status);

    void request_state1(uint8_t index); // 0x9A
    void request_state2(uint8_t index); // 0x9C
    void request_encoder(uint8_t index); // 0x92

    void get_state_all();

private:
    CAN &_can;
    uint8_t _motorCount;

    MotorState _status[8];

    uint16_t getCanID(uint8_t motorID) const;
    void sendCommand(uint8_t motorID, uint8_t cmd, const uint8_t data[8]);
};

#endif