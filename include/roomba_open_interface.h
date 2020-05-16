#pragma once

#include <cstdint>

namespace Roomba{
enum class OpCode : uint8_t {
    START = 128,
    RESET = 7,
    STOP = 173,
    BAUD = 129,
    CONTROL = 130,
    SAFE = 131,
    FULL = 132,
    CLEAN = 135,
    MAX = 136,
    SPOT = 134,
    DOCK = 143,
    POWER = 133,
    SCHEDULE = 167,
    DATE = 168,
    DRIVE = 137,
    DRIVE_DIRECT = 145,
    DRIVE_PWM = 146,
    MOTORS = 138,
    MOTORS_PWM = 144,
    LEDS = 139,
    SCHEDULING_LEDS = 162,
    DIGIT_LEDS_RAW = 163,
    BUTTONS = 165,
    DIGIT_LEDS_ASCII = 164,
    SONG = 140,
    PLAY = 141,
    SENSORS= 142,
    QUERY_LIST=149,
    STREAM = 148,
    TOGGLE_STREAM = 150
};


enum class BAUDCODE : uint8_t {
    BAUD_300 = 0,
    BAUD_600 = 1,
    BAUD_1200 = 2,
    BAUD_2400 = 3,
    BAUD_4800 = 4,
    BAUD_9600 = 5,
    BAUD_14400 = 6,
    BAUD_19200 = 7,
    BAUD_28800 = 8,
    BAUD_38400 = 9,
    BAUD_57600 = 10,
    BAUD_115200 = 11
};

// According to the Create OI spec the values are following
// PASSIVE =1, SAVE = 2, FULL = 3 but reading the port values
// for my robot provides the following values
enum class OIMode : int8_t{
    OFF = 0,
    PASSIVE = 2,
    SAFE = 3,
    FULL = 4,
    UNAVAILABLE = -1
};

enum class ChargingState : uint8_t{
    CHARGE_NONE = 0,
    CHARGE_RECONDITION = 1,
    CHARGE_FULL = 2,
    CHARGE_TRICKLE = 3,
    CHARGE_WAITING = 4,
    CHARGE_FAULT = 5
};

namespace utils{
inline int16_t From2sComplement(uint8_t bytes[2]){
    int16_t value = 0;
    value = ((int16_t) bytes[0] << 8) & 0xFF00 | (int16_t) bytes[1];
    return value;
}

inline void To2sComplementBytes(int16_t int_val, uint8_t bytes[2]){
    uint16_t high = int_val & 0xFF00;
    bytes[0] = high >> 8;
    bytes[1] = int_val & 0xFF;
}
}

} // end of namespace Roomba
