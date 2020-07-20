#pragma once

#include <string>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>
#include <map>

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

const std::map<PacketId, uint8_t> PacketSizeList = {
    // Group
    {PacketId::GROUP_6, 52},
    {PacketId::GROUP_100, 80},
    {PacketId::GROUP_101, 28},
    {PacketId::NUM_STREAM_PACKETS, 1},


    // State
    {PacketId::OI_MODE, 1},
    {PacketId::LIGHT, 1},
    {PacketId::BUMP_WHEELDROP, 1},

    // Battery related
    {PacketId::CHARGE_STATE, 1},
    {PacketId::TEMPERATURE, 1},
    {PacketId::VOLTAGE, 2},
    {PacketId::CURRENT, 2},
    {PacketId::CAPACITY, 2},
    {PacketId::CHARGE, 2},
    {PacketId::CHARGE_SOURCE, 1},

    // Motion
    {PacketId::DISTANCE, 2},
    {PacketId::ANGLE, 2},
    {PacketId::RIGHT_ENC, 2},
    {PacketId::LEFT_ENC, 2},
    {PacketId::VEL, 2},
    {PacketId::RADIUS, 2},
    {PacketId::RIGHT_VEL, 2},
    {PacketId::LEFT_VEL, 2},

    // Bumper state
    {PacketId::LIGHT_RIGHT, 2},
    {PacketId::LIGHT_FRONT_RIGHT, 2},
    {PacketId::LIGHT_CENTER_RIGHT, 2},
    {PacketId::LIGHT_CENTER_LEFT, 2},
    {PacketId::LIGHT_FRONT_LEFT, 2},
    {PacketId::LIGHT_LEFT, 2},

    // Wall & Cliff
    {PacketId::VIRTUAL_WALL, 1},
    {PacketId::WALL_SIGNAL, 2},
    {PacketId::CLIFF_LEFT_SIGNAL, 2},
    {PacketId::CLIFF_FRONT_LEFT_SIGNAL, 2},
    {PacketId::CLIFF_FRONT_RIGHT_SIGNAL, 2},
    {PacketId::CLIFF_RIGHT_SIGNAL, 2},


};

struct Packet{
    uint8_t* const getId() const{
        return (uint8_t*)&id;
    }
    PacketId getPktId() const {
        return id;
    }
    uint8_t* getDataPtr(){
        return data;
    }
    int getDataSize() const {
        return PacketSizeList.at(id);
    }
    virtual std::string toString() const {
        std::ostringstream ss;
        ss << (int) *data;
        return ss.str();
    }

    Packet& operator=(const Packet& other){
        id = other.id;
        memcpy(data, other.data, PacketSizeList.at(id));
        return *this;
    }
    Packet() = delete;
    virtual ~Packet() {
        if (m_data_allocated){
            free(data);
        }
    }
protected:
    // Can't make the id constant as that will prevent the assignment
    // or copy construction
    PacketId id;
    explicit Packet(PacketId pkt_id) : id(pkt_id){
        data =
            reinterpret_cast<uint8_t*>(
                malloc(sizeof(uint8_t)*PacketSizeList.at(id)));
        m_data_allocated = true;
    }
    explicit Packet(PacketId pkt_id, uint8_t* data_ptr)
        : id(pkt_id)
        , data(data_ptr)
        , m_data_allocated(false){
        // Empty
    }

    bool m_data_allocated{false};
    uint8_t* data{nullptr};
};

struct GroupPkt : public Packet {
    const Packet* getPacket(PacketId id) const{
        return m_PktList.at(id).get();
    }
protected:
    GroupPkt(PacketId id) : Packet(id){
    }
    GroupPkt(PacketId id, uint8_t* data_ptr)
        : Packet(id, data_ptr){
    }
    std::map<PacketId, std::unique_ptr<Packet>> m_PktList;
};

struct Group6Pkt : public GroupPkt {
    Group6Pkt();
    Group6Pkt(uint8_t* data_ptr);
    std::string toString() const override;

    // The following methods are useful for logging
    static std::string LogHeaderStr();
    static std::string LogDataStr(Group6Pkt* pkt);

private:
    void setup();
};

struct Group100Pkt : public GroupPkt {
    Group100Pkt() : GroupPkt(PacketId::GROUP_100){
    }
    Group100Pkt(uint8_t* data_ptr)
        : GroupPkt(PacketId::GROUP_100, data_ptr){
    }
    std::string toString() const override;
};

struct Group101Pkt : public GroupPkt {
    Group101Pkt() : GroupPkt(PacketId::GROUP_101){
    }
    Group101Pkt(uint8_t* data_ptr)
        : GroupPkt(PacketId::GROUP_101, data_ptr){
    }
    std::string toString() const override;
};

struct UnsignedValuePkt : public Packet{
    uint16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    virtual std::string toString() const override {
        std::ostringstream ss;
        ss << getValue();
        return ss.str();
    }
protected:
    UnsignedValuePkt(PacketId id) : Packet(id){
    }
    UnsignedValuePkt(PacketId id, uint8_t* data_ptr)
        : Packet(id, data_ptr){
    }
};

struct SignedValuePkt : public Packet{
    int16_t getValue() const{
        return Roomba::utils::From2sComplement(data);
    }
    virtual std::string toString() const override {
        std::ostringstream ss;
        ss << getValue();
        return ss.str();
    }
protected:
    SignedValuePkt(PacketId id) : Packet(id){
    }
    SignedValuePkt(PacketId id, uint8_t* data_ptr)
        : Packet(id, data_ptr){
    }
};

struct OIMode : public Packet{
    OIMode() : Packet(PacketId::OI_MODE){
    }
    OIMode(uint8_t* data_ptr)
        : Packet(PacketId::OI_MODE, data_ptr){
    }

    Roomba::OIMode getValue(){
        return (Roomba::OIMode) *data;
    }
    std::string toString() const override {
        Roomba::OIMode mode = (Roomba::OIMode) *data;
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
                return "OI Mode: UNAVAILABLE";
            break;
        }
    }
};

struct EncoderLeft : public SignedValuePkt{
    EncoderLeft() : SignedValuePkt(PacketId::LEFT_ENC){
    }
    EncoderLeft(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::LEFT_ENC, data_ptr){
    }
};

struct EncoderRight : public SignedValuePkt{
    EncoderRight() : SignedValuePkt(PacketId::RIGHT_ENC){
    }
    EncoderRight(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::RIGHT_ENC, data_ptr){
    }
};

struct DistanceTravelled : public SignedValuePkt{
    DistanceTravelled() : SignedValuePkt(PacketId::DISTANCE){
    }
    DistanceTravelled(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::DISTANCE, data_ptr){
    }

    std::string toString() const override {
        std::ostringstream ss;
        ss << "Distance travelled: " << getValue() << " mm";
        return ss.str();
    }
};

struct AngleTurned : public SignedValuePkt{
    AngleTurned() : SignedValuePkt(PacketId::ANGLE){
    }
    AngleTurned(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::ANGLE, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Angle turned: " << getValue() << " degrees";
        return ss.str();
    }
};

struct VelocityRequested : public SignedValuePkt{
    VelocityRequested() : SignedValuePkt(PacketId::VEL){
    }
    VelocityRequested(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::VEL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Requested Velocity: " << getValue() << " mm/s";
        return ss.str();
    }
};

struct RadiusRequested : public SignedValuePkt{
    RadiusRequested() : SignedValuePkt(PacketId::RADIUS){
    }
    RadiusRequested(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::RADIUS, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Requested radius: " << getValue() << " mm";
        return ss.str();
    }
};

struct RequestedRightVelocity : public SignedValuePkt{
    RequestedRightVelocity() : SignedValuePkt(PacketId::RIGHT_VEL){
    }
    RequestedRightVelocity(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::RIGHT_VEL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Requested Right Velocity: " << getValue() << " mm/s";
        return ss.str();
    }
};

struct RequestedLeftVelocity : public SignedValuePkt{
    RequestedLeftVelocity() : SignedValuePkt(PacketId::LEFT_VEL){
    }
    RequestedLeftVelocity(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::LEFT_VEL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Requested Left Velocity: " << getValue() << " mm/s";
        return ss.str();
    }
};

//------------------------------------------------------------------------------
// Cliff signals
//------------------------------------------------------------------------------

struct WallSignal : public UnsignedValuePkt{
    WallSignal() : UnsignedValuePkt(PacketId::WALL_SIGNAL){
    }
    WallSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::WALL_SIGNAL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Wall signal: " << getValue();
        return ss.str();
    }
};

struct CliffLeftSignal : public UnsignedValuePkt{
    CliffLeftSignal() : UnsignedValuePkt(PacketId::CLIFF_LEFT_SIGNAL){
    }
    CliffLeftSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CLIFF_LEFT_SIGNAL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Left: " << getValue();
        return ss.str();
    }
};

struct CliffFrontLeftSignal : public UnsignedValuePkt{
    CliffFrontLeftSignal() : UnsignedValuePkt(PacketId::CLIFF_FRONT_LEFT_SIGNAL){
    }
    CliffFrontLeftSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CLIFF_FRONT_LEFT_SIGNAL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Front Left: " << getValue();
        return ss.str();
    }
};

struct CliffFrontRightSignal : public UnsignedValuePkt{
    CliffFrontRightSignal() : UnsignedValuePkt(PacketId::CLIFF_FRONT_RIGHT_SIGNAL){
    }
    CliffFrontRightSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CLIFF_FRONT_RIGHT_SIGNAL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Front Right: " << getValue();
        return ss.str();
    }
};

struct CliffRightSignal : public UnsignedValuePkt{
    CliffRightSignal() : UnsignedValuePkt(PacketId::CLIFF_RIGHT_SIGNAL){
    }
    CliffRightSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CLIFF_RIGHT_SIGNAL, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Cliff Right: " << getValue();
        return ss.str();
    }
};

//------------------------------------------------------------------------------
//  Bump Signals
//------------------------------------------------------------------------------
struct LightBumpRightSignal : public UnsignedValuePkt{
    LightBumpRightSignal() : UnsignedValuePkt(PacketId::LIGHT_RIGHT){
    }
    LightBumpRightSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_RIGHT, data_ptr){
    }
};

struct LightBumpFrontRightSignal : public UnsignedValuePkt{
    LightBumpFrontRightSignal() : UnsignedValuePkt(PacketId::LIGHT_FRONT_RIGHT){
    }
    LightBumpFrontRightSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_FRONT_RIGHT, data_ptr){
    }
};
struct LightBumpCenterRightSignal : public UnsignedValuePkt{
    LightBumpCenterRightSignal() : UnsignedValuePkt(PacketId::LIGHT_CENTER_RIGHT){
    }
    LightBumpCenterRightSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_CENTER_RIGHT, data_ptr){
    }
};
struct LightBumpCenterLeftSignal : public UnsignedValuePkt{
    LightBumpCenterLeftSignal() : UnsignedValuePkt(PacketId::LIGHT_CENTER_LEFT){
    }
    LightBumpCenterLeftSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_CENTER_LEFT, data_ptr){
    }
};
struct LightBumpFrontLeftSignal : public UnsignedValuePkt{
    LightBumpFrontLeftSignal() : UnsignedValuePkt(PacketId::LIGHT_FRONT_LEFT){
    }
    LightBumpFrontLeftSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_FRONT_LEFT, data_ptr){
    }
};

struct LightBumpLeftSignal : public UnsignedValuePkt{
    LightBumpLeftSignal() : UnsignedValuePkt(PacketId::LIGHT_LEFT){
    }
    LightBumpLeftSignal(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::LIGHT_LEFT, data_ptr){
    }
};

struct LightBumper : public Packet{
    LightBumper() : Packet(PacketId::LIGHT){
    }
    uint8_t getValue() const{
        return *data;
    }
    std::string toString() const override {
        char data[] = "............";
        uint8_t state = getValue();

        if (state & 0x1){
            data[10] = 'L';
        }
        if (state & 0x2){
            data[9] = 'F';
            data[8] = 'L';
        }
        if (state & 0x4){
            data[7] = 'C';
            data[6] = 'L';
        }
        if (state & 0x8){
            data[5] = 'C';
            data[4] = 'R';
        }
        if (state & 0x10){
            data[3] = 'F';
            data[2] = 'R';
        }
        if (state & 0x20){
            data[1] = 'R';
            data[0] = '.';
        }

        return data;
    }
};

//------------------------------------------------------------------------------
//  Battery related
//------------------------------------------------------------------------------

struct ChargingState : public Packet{
    ChargingState() : Packet(PacketId::CHARGE_STATE){
    }
    ChargingState(uint8_t* data_ptr)
        : Packet(PacketId::CHARGE_STATE, data_ptr){
    }
    std::string toString() const override {
        switch (*data){
            case 0:
                return "Not charging";
            break;
            case 1:
                return "Reconditioning Charging";
            break;
            case 2:
                return "Full Charging";
            break;
            case 3:
                return "Trickle Charging";
            break;
            case 4:
                return "Waiting";
            break;
            case 5:
                return "Charging Fault Condition";
            break;
        }
        return "Unknown";
    }
};


struct Voltage : public SignedValuePkt{
    Voltage() : SignedValuePkt(PacketId::VOLTAGE){
    }
    Voltage(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::VOLTAGE, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery voltage: " << getValue()/1000.0 << " V";
        return ss.str();
    }
};

struct Current : public SignedValuePkt{
    Current() : SignedValuePkt(PacketId::CURRENT){
    }
    Current(uint8_t* data_ptr)
        : SignedValuePkt(PacketId::CURRENT, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery current: " << getValue() << " mA";
        return ss.str();
    }
};

struct Temperature : public Packet{
    Temperature() : Packet(PacketId::TEMPERATURE){
    }
    Temperature(uint8_t* data_ptr)
        : Packet(PacketId::TEMPERATURE, data_ptr){
    }
    int8_t getValue() const{
        return (int8_t) *data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss
            << "Battery temperature: " << std::to_string(getValue())
            << " C";
        return ss.str();
    }
};

struct BatteryCapacity : public UnsignedValuePkt{
    BatteryCapacity() : UnsignedValuePkt(PacketId::CAPACITY){
    }
    BatteryCapacity(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CAPACITY, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery capacity: " << getValue() << " mAh";
        return ss.str();
    }
};

struct BatteryCharge : public UnsignedValuePkt{
    BatteryCharge() : UnsignedValuePkt(PacketId::CHARGE){
    }
    BatteryCharge(uint8_t* data_ptr)
        : UnsignedValuePkt(PacketId::CHARGE, data_ptr){
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Battery charge: " << getValue() << " mAh";
        return ss.str();
    }
};

struct ChargingSourceAvailable : public Packet{
    ChargingSourceAvailable() : Packet(PacketId::CHARGE_SOURCE){
    }
    ChargingSourceAvailable(uint8_t* data_ptr)
        : Packet(PacketId::CHARGE_SOURCE, data_ptr){
    }
    uint8_t getValue() const{
        return *data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Charging Source: " << (int)*data;
        return ss.str();
    }
};

struct BumpAndWheelDrop : public Packet{
    BumpAndWheelDrop() : Packet(PacketId::BUMP_WHEELDROP){
    }
    BumpAndWheelDrop(uint8_t* data_ptr)
        : Packet(PacketId::BUMP_WHEELDROP, data_ptr){
    }
    uint8_t getValue() const{
        return (uint8_t) *data;
    }
    std::string toString() const override {
        char data[] = "0000";
        uint8_t state = getValue();
        for (int i=0; i < sizeof(data); i++){
            if (state & (0x1 << i)){
                data[i] = '1';
            }
        }
        return data;
    }
};

struct OiStreamNumPkts : public Packet{
    OiStreamNumPkts() : Packet(PacketId::NUM_STREAM_PACKETS){
    }
    OiStreamNumPkts(uint8_t* data_ptr)
        : Packet(PacketId::NUM_STREAM_PACKETS, data_ptr){
    }
    uint8_t getValue() const{
        return (uint8_t) *data;
    }
    std::string toString() const override {
        std::ostringstream ss;
        ss << "Number of Streamed packets: " << (int) getValue();
        return ss.str();
    }
};


} // namespace Sensor
} // namespace Roomba