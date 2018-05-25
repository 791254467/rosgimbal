#ifndef GIMBAL_H
#define GIMBAL_H

// serial communication
#define OUT_BUFFER_SIZE 22
#define OUT_START_BYTE 0xA5
#define OUT_PAYLOAD_LENGTH 20
#define OUT_MESSAGE_LENGTH 22

#define IN_BUFFER_SIZE 14
#define IN_START_BYTE 0xA5
#define IN_PAYLOAD_LENGTH 12
#define IN_MESSAGE_LENGTH 14

#define CRC_LENGTH 1
#define CRC_INITIAL_VALUE 0x00

#include "system.h"
#include "pwm.h"
#include "led.h"
#include "vcp.h"

#include "revo_f4.h"

namespace gimbal {

class Gimbal
{
public:
    Gimbal();

    enum ParseState {
        PARSE_STATE_IDLE,
        PARSE_STATE_GOT_START_BYTE,
        PARSE_STATE_GOT_PAYLOAD
    };

    ParseState parse_state;

    void rx_callback(uint8_t byte);

private:
//    VCP* uartPtr = NULL;
    bool parse_in_byte(uint8_t c);

    void handle_in_msg(float roll, float pitch, float yaw);
    void unpack_in_payload(uint8_t buf[IN_PAYLOAD_LENGTH], float *roll, float *pitch, float *yaw);

    uint8_t in_crc8_ccitt_update (uint8_t inCrc, uint8_t inData);
    uint8_t out_crc8_ccitt_update (uint8_t outCrc, uint8_t outData);

    void blink_led();
    void norm_commands();
    void calc_command_rate();

//    void rx_callback(uint8_t byte);
    void tx_callback(float command_rate, float servo_rate, float roll, float pitch, float yaw);

    volatile float roll_command;
    volatile float pitch_command;
    volatile float yaw_command;
    volatile float norm_roll;
    volatile float norm_pitch;
    volatile float norm_yaw;

    // This value is the total range in radians the servo can travel.
    float RAD_RANGE = 3.14159;

    // Offset values for aligning gimbal servos with desired coordinate frame.
    // Radians. Must be positive.
    float roll_offset = 0;
    float pitch_offset = -0.45;
    float yaw_offset = 0;

    // Limit the servo travel for mechanical restrictions.
    // These are normalized values for the write function. Between 0 and 1.
    float roll_lower_limit = 0;
    float pitch_lower_limit = 0;
    float yaw_lower_limit = 0;

    float roll_upper_limit = 1.0;
    float pitch_upper_limit = 2.15/RAD_RANGE;
    float yaw_upper_limit = 1.0;

    long time_of_last_command;
    long time_of_last_blink;
    long time_of_last_servo;

    uint32_t crc_error_count;
    uint32_t start_byte_error;
    uint32_t payload_index_error;
    float command_in_rate;
    float servo_command_rate;

//    ParseState parse_state;

    // serial
    uint8_t out_buf[OUT_BUFFER_SIZE];
    uint8_t in_buf[IN_BUFFER_SIZE];

    uint8_t in_payload_buf[IN_PAYLOAD_LENGTH];
    int in_payload_index;
    uint8_t in_crc_value;
    uint8_t out_crc_value;

    LED info;



protected:


};

#endif // GIMBAL_H

} // End gimbal namespace
