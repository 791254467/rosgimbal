#ifndef GIMBAL_H
#define GIMBAL_H

#include "system.h"
#include "pwm.h"
#include "led.h"
#include "vcp.h"

#include "revo_f4.h"


namespace gimbal
{

class Gimbal
{
public:
    // Functions
    Gimbal();
    void rx_callback(uint8_t byte);
    void norm_commands();
    void tx_callback(float command_rate, float servo_rate, float roll, float pitch, float yaw);
    void calc_servo_rate();


    // Variables

    // serial communication
    static constexpr int OUT_BUFFER_SIZE = 22;
    static constexpr int OUT_START_BYTE = 0xA5;
    static constexpr int OUT_PAYLOAD_LENGTH = 20;
    static constexpr int OUT_MESSAGE_LENGTH = 22;

    static constexpr int IN_BUFFER_SIZE = 14;
    static constexpr int IN_START_BYTE = 0xA5;
    static constexpr int IN_PAYLOAD_LENGTH = 12;
    static constexpr int IN_MESSAGE_LENGTH = 14;

    static constexpr int CRC_LENGTH = 1;
    static constexpr int CRC_INITIAL_VALUE = 0x00;

    // This value is the total range in radians the servo can travel.
    static constexpr float RAD_RANGE = 3.14159;


    // Offset values for aligning gimbal servos with desired coordinate frame.
    // Radians. Must be positive.
    static constexpr float roll_offset = 0;
    static constexpr float pitch_offset = -0.45;
    static constexpr float yaw_offset = 0;

    volatile float roll_command;
    volatile float pitch_command;
    volatile float yaw_command;
    volatile float norm_roll;
    volatile float norm_pitch;
    volatile float norm_yaw;

    volatile float command_in_rate;
    volatile float servo_command_rate;
    volatile long time_of_last_servo;

    uint8_t out_buf[OUT_BUFFER_SIZE];

private:

    // Functions
    void handle_in_msg(float roll, float pitch, float yaw);
    void unpack_in_payload(uint8_t buf[], float *roll, float *pitch, float *yaw);
    bool parse_in_byte(uint8_t c);
    uint8_t in_crc8_ccitt_update (uint8_t inCrc, uint8_t inData);
    uint8_t out_crc8_ccitt_update (uint8_t outCrc, uint8_t outData);
    void blink_led();
    void calc_command_rate();

    // Variables
    enum ParseState {
        PARSE_STATE_IDLE,
        PARSE_STATE_GOT_START_BYTE,
        PARSE_STATE_GOT_PAYLOAD
    };

    // Limit the servo travel for mechanical restrictions.
    // These are normalized values for the write function. Between 0 and 1.
    static constexpr float roll_lower_limit = 0;
    static constexpr float pitch_lower_limit = 0;
    static constexpr float yaw_lower_limit = 0;

    static constexpr float roll_upper_limit = 1.0;
    static constexpr float pitch_upper_limit = 2.15/RAD_RANGE;
    static constexpr float yaw_upper_limit = 1.0;

    volatile long time_of_last_command;
    volatile long time_of_last_blink;


    volatile uint32_t crc_error_count;
    volatile uint32_t start_byte_error;
    volatile uint32_t payload_index_error;



    // serial
    volatile uint8_t in_buf[IN_BUFFER_SIZE];

    ParseState parse_state;
    uint8_t in_payload_buf[IN_PAYLOAD_LENGTH];
    volatile int in_payload_index;
    volatile uint8_t in_crc_value;
    volatile uint8_t out_crc_value;

    LED info;

protected:

};

#endif // GIMBAL_H
} // end gimbal namespace
