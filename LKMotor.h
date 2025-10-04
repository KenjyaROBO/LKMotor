#pragma once

#include "mbed.h"
#include <cstdint>

// モーターの状態を格納する構造体
typedef struct {
    int8_t temp;
    int16_t trq_cur;
    int16_t spd;
    int16_t deg;
    uint16_t volt;
    int64_t total_deg;
    bool updated9A;
    bool updated9C;
    bool updated92;
} LKMState;

class LKMotor {
public:
    LKMotor(CAN &can, uint8_t motor_count);
    
    // 制御コマンド
    void motorOn();
    void motorOff();
    void stopMotor();
    void trq_control(int motor_index, int16_t torqueControl);
    void trq_controlAll(int16_t* torques);
    void spd_control(int motor_index, int32_t speedControl);
    void spd_controlAll(int32_t* speeds);
    void angle_control(int motor_index, int32_t angleControl);
    void angle_controlAll(int32_t* angles);
    void set_angle_control(int motor_index, int32_t angleControl, uint16_t maxSpeed);
    void set_angle_controlAll(int32_t* angles,uint16_t* speeds);
    void abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection);
    void abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections);
    void set_abs_angle_control(int motor_index, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed);
    void set_abs_angle_controlAll(uint32_t* angles,uint8_t* spinDirections, uint16_t* maxSpeeds);
    void inc_angle_control(int motor_index, int32_t angleIncrement);
    void inc_angle_controlAll(int32_t* angleIncrements);
    void set_inc_angle_control(int motor_index, int32_t angleIncrement, uint16_t maxSpeed);
    void set_inc_angle_controlAll(int32_t* angleIncrements, uint16_t* maxSpeeds);
    
    // 状態要求コマンド
    void request_state1(uint8_t index);
    void request_state1All();
    void request_state2(uint8_t index);
    void request_state2All();
    void request_encoder(uint8_t index);
    void request_encoderAll();
    void request(uint8_t index);
    void requestAll();

    // CANメッセージを処理する関数 (変更なし)
    void get_msg(const CANMessage &msg);

    bool get_state(uint8_t index, LKMState &status);

    LKMState read(uint8_t index) const;

private:
    CAN &_can;
    uint8_t _motorCount;
    LKMState _status[8];
    int count; // request関数で使用

    uint16_t getCanID(uint8_t motorID) const;
    void sendCommand(uint8_t motorID, uint8_t cmd, const uint8_t data[8]);
};
