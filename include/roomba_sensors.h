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
    TEMPERATURE = 24,
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
    uint8_t* const getId() const{
        return (uint8_t*)&id;
    }
    virtual uint8_t* getDataPtr() = 0;
    virtual int getDataSize() const = 0;
    virtual std::string toString() const = 0;
    virtual ~Packet() {
        // Empty
    }
    // static std::unique_ptr<Packet> NewInstance(PacketId id){

    // }
protected:
    PacketId id;
    Packet(PacketId pkt_id) : id(pkt_id){
        // Empty
    }
};

struct OneBytePacket : public Packet{
    uint8_t* getDataPtr(){
        return reinterpret_cast<uint8_t*>(&data);
    }
    int getDataSize() const override{
        return 1;
    }
    virtual ~OneBytePacket() {
        // Empty
    }
    OneBytePacket& operator=(const OneBytePacket& other){
        Packet::id = other.id;
        data = other.data;
        return *this;
    }
protected:
    OneBytePacket(PacketId pkt_id) : Packet(pkt_id){
        // Empty
    }
    int8_t data{0};
};

struct TwoBytePacket : public Packet{
    uint8_t* getDataPtr(){
        return reinterpret_cast<uint8_t*>(&data);
    }
    int getDataSize() const override{
        return 2;
    }
    virtual ~TwoBytePacket() {
        // Empty
    }
    TwoBytePacket& operator=(const TwoBytePacket& other){
        Packet::id = other.id;
        data[0] = other.data[0];
        data[1] = other.data[1];
        return *this;
    }

protected:
    TwoBytePacket(PacketId pkt_id) : Packet(pkt_id){
        // Empty
    }
    uint8_t data[2]{0};
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

struct OIMode : public OneBytePacket{
    OIMode() : OneBytePacket(PacketId::OI_MODE){
    }
    uint8_t* getDataPtr() override{
        return reinterpret_cast<uint8_t*>(&data);
    }

    std::string toString() const override {
        Roomba::OIMode mode = (Roomba::OIMode) data;
        switch (mode){
            case Roomba::OIMode::OFF:
                return "OI Mode: OFF";
            break;
            case Roomba::OIMode::PASSIVE:
                return "OI Mode: PASSIVE";
            break;
            case Roomba::OIMode::SAFE:
                return "OI Mode: SAFE";
            break;
            case Roomba::OIMode::FULL:
                return "OI Mode: FULL";
            break;
            default:
                return "OI Mode: UNKNOWN";
            break;
        }
    }
    Roomba::OIMode data{Roomba::OIMode::UNAVAILABLE};
};

struct EncoderLeft : public TwoBytePacket{
    EncoderLeft() : TwoBytePacket(PacketId::DISTANCE){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Encoder Left: " << getValue() << " mm";
        return ss.str();
    }
};

struct EncoderRight : public TwoBytePacket{
    EncoderRight() : TwoBytePacket(PacketId::DISTANCE){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Encoder Right: " << getValue() << " mm";
        return ss.str();
    }
};


struct DistanceTravelled : public TwoBytePacket{
    DistanceTravelled() : TwoBytePacket(PacketId::DISTANCE){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Distance travelled: " << getValue() << " mm";
        return ss.str();
    }
};

struct AngleTurned : public TwoBytePacket{
    AngleTurned() : TwoBytePacket(PacketId::ANGLE){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Angle turned: " << getValue() << " degree";
        return ss.str();
    }
};

struct CliffLeft : public OneBytePacket{
    CliffLeft() : OneBytePacket (PacketId::CLIFF_LEFT){
    }
    uint8_t getValue() const{
        return (uint8_t)data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Left: " << (int) getValue();
        return ss.str();
    }
};

struct CliffRight : public OneBytePacket{
    CliffRight() : OneBytePacket(PacketId::CLIFF_RIGHT){
    }
    uint8_t getValue() const{
        return (uint8_t)data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Right: " << (int) getValue();
        return ss.str();
    }
};

struct LightBumper : public OneBytePacket{
    LightBumper() : OneBytePacket(PacketId::LIGHT){
    }
    uint8_t getValue() const{
        return data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Light Bumper: 0x" << std::hex << (int) getValue();
        return ss.str();
    }
};


struct Voltage : public TwoBytePacket{
    Voltage() : TwoBytePacket(PacketId::VOLTAGE){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery voltage: " << getValue()/1000.0 << " V";
        return ss.str();
    }
};

struct Current : public TwoBytePacket{
    Current() : TwoBytePacket(PacketId::CURRENT){
    }
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery current: " << getValue() << " mA";
        return ss.str();
    }
};

struct BatteryCapacity : public TwoBytePacket{
    BatteryCapacity() : TwoBytePacket(PacketId::CAPACITY){
    }
    uint16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery capacity: " << getValue() << " mAh";
        return ss.str();
    }
};

struct Temperature : public OneBytePacket{
    Temperature() : OneBytePacket(PacketId::TEMPERATURE){
    }
    int8_t getValue() const{
        return (int8_t) data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery temperature: " << std::to_string(getValue()) << " C";
        return ss.str();
    }
};

} // namespace Sensor
} // namespace Roomba