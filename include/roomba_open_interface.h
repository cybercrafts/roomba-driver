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

  enum class SensorPacketID : uint8_t {
    GROUP_0 = 0,
    GROUP_1 = 1,
    GROUP_2 = 2,
    GROUP_3 = 3,
    GROUP_4 = 4,
    GROUP_5 = 5,
    GROUP_6 = 6,
    GROUP_100 = 100,
    GROUP_101 = 101,
    GROUP_106 = 106,
    GROUP_107 = 107,
    BUMP_WHEELDROP = 7,
    WALL = 8,
    CLIFF_LEFT = 9,
    CLIFF_FRONT_LEFT = 10,
    CLIFF_FRONT_RIGHT = 11,
    CLIFF_RIGHT = 12,
    VIRTUAL_WALL = 13,
    OVERCURRENTS = 14,
    DIRT_DETECT_LEFT = 15,
    DIRT_DETECT_RIGHT = 16,
    IR_OMNI = 17,
    IR_LEFT = 52,
    IR_RIGHT = 53,
    BUTTONS = 18,
    DISTANCE = 19,
    ANGLE = 20,
    CHARGE_STATE = 21,
    VOLTAGE = 22,
    CURRENT = 23,
    TEMP = 24,
    CHARGE = 25,
    CAPACITY = 26,
    WALL_SIGNAL = 27,
    CLIFF_LEFT_SIGNAL = 28,
    CLIFF_FRONT_LEFT_SIGNAL = 29,
    CLIFF_FRONT_RIGHT_SIGNAL = 30,
    CLIFF_RIGHT_SIGNAL = 31,
    CARGO_BAY_DIGITAL_INPUTS = 32,
    CARGO_BAY_ANALOG_SIGNAL = 33,
    CHARGE_SOURCE = 34,
    OI_MODE = 35,
    SONG_NUM = 36,
    PLAYING = 37,
    NUM_STREAM_PACKETS = 38,
    VEL = 39,
    RADIUS = 40,
    RIGHT_VEL = 41,
    LEFT_VEL = 42,
    LEFT_ENC = 43,
    RIGHT_ENC = 44,
    LIGHT = 45,
    LIGHT_LEFT = 46,
    LIGHT_FRONT_LEFT = 47,
    LIGHT_CENTER_LEFT = 48,
    LIGHT_CENTER_RIGHT = 49,
    LIGHT_FRONT_RIGHT = 50,
    LIGHT_RIGHT = 51,
    LEFT_MOTOR_CURRENT = 54,
    RIGHT_MOTOR_CURRENT = 55,
    MAIN_BRUSH_CURRENT = 56,
    SIDE_BRUSH_CURRENT = 57,
    STASIS = 58,
    NUM = 52
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
    PASSIVE = 1,
    SAFE = 2,
    FULL = 3,
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

} // end of namespace Roomba
