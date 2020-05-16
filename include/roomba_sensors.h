#pragma once

#include <string>
#include <sstream>
#include <cstdint>
#include <cstring>

#include "roomba_open_interface.h"

namespace Roomba{
namespace Sensor{
enum class PacketId : uint8_t {
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


struct Packet{
    virtual uint8_t* getId() = 0;
    virtual uint8_t* getData() = 0;
    virtual int getDataSize() = 0;
    virtual std::string toString() = 0;
    virtual ~Packet() {
        // Empty
    }
    PacketId id;
};

// struct BumpAndWheelDrop : public Packet{
//     PacketId getId() override{
//         return PacketId::BUMP_WHEELDROP;
//     }

//     uint8_t data{0};
// };

// struct CliffLeft : public Packet{
//     PacketId getId() override{
//         return PacketId::CLIFF_LEFT;
//     }
//     uint8_t data{0};
// };

struct OIMode : public Packet{
    OIMode(){
        id = PacketId::OI_MODE;
    }
    uint8_t* getId() override{
        return reinterpret_cast<uint8_t*>(&id);
    }
    uint8_t* getData() override{
        return reinterpret_cast<uint8_t*>(&data);
    }
    int getDataSize() override {
        return 1;
    }

    std::string toString() override {
        Roomba::OIMode mode = (Roomba::OIMode) data;
        switch (mode){
            case Roomba::OIMode::OFF:
                return "OFF";
            break;
            case Roomba::OIMode::PASSIVE:
                return "PASSIVE";
            break;
            case Roomba::OIMode::SAFE:
                return "SAFE";
            break;
            case Roomba::OIMode::FULL:
                return "FULL";
            break;
            default:
                return "UNKNOWN";
            break;
        }
    }
    Roomba::OIMode data{Roomba::OIMode::UNAVAILABLE};
};


struct DistanceTravelled : public Packet{
    DistanceTravelled(){
        id = PacketId::DISTANCE;
    }
    uint8_t* getId() override{
        return reinterpret_cast<uint8_t*>(&id);
    }
    uint8_t* getData() override{
        return reinterpret_cast<uint8_t*>(&data);
    }
    int getDataSize() override {
        return 2;
    }
    int16_t getValue(){
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() override {
        std::ostringstream ss;
        ss << getValue() << " mm";
        return ss.str();
    }
    uint8_t data[2]{0};
};

struct AngleTurned : public Packet{
    AngleTurned(){
        id = PacketId::ANGLE;
    }
    uint8_t* getId() override{
        return reinterpret_cast<uint8_t*>(&id);
    }
    uint8_t* getData() override{
        return reinterpret_cast<uint8_t*>(&data);
    }
    int getDataSize() override {
        return 2;
    }
    int16_t getValue(){
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() override {
        std::ostringstream ss;
        ss << getValue() << " degree";
        return ss.str();
    }
    uint8_t data[2]{0};
};

} // namespace Sensor
} // namespace Roomba