#include "roomba_sensors.h"
#include <iostream>

using namespace std;

namespace Roomba{
namespace Sensor{

Group6Pkt::Group6Pkt()
    : GroupPkt(PacketId::GROUP_6){

    // Setup
    setup();
}
Group6Pkt::Group6Pkt(uint8_t* data_ptr)
    : GroupPkt(PacketId::GROUP_6, data_ptr){

    // Setup
    setup();
}

std::string Group6Pkt::toString() const {
    std::ostringstream ss;
    for(auto const& items : m_PktList){
        //cout << "NEXT\n" << items.second->toString() << endl;
        ss << items.second->toString() << endl;
    }
    return ss.str();
}

void Group6Pkt::setup(){

    // TODO
    // Replace all these "new"s with preallocatd object
    // Essentially provide Serialize methods

    uint8_t* data_ptr = getDataPtr();

    int offset = 0;
    unique_ptr<Packet> nextPkt(new BumpAndWheelDrop(data_ptr+offset));
    m_PktList[PacketId::BUMP_WHEELDROP] = std::move(nextPkt);

    // Virtual wall
    offset = 6;
    // TODO

    // Dirt detect
    offset = 8;
    // TODO

    offset = 12;
    nextPkt = unique_ptr<Packet>(new DistanceTravelled(data_ptr+offset));
    m_PktList[PacketId::DISTANCE] = std::move(nextPkt);

    offset = 14;
    nextPkt = unique_ptr<Packet>(new AngleTurned(data_ptr+offset));
    m_PktList[PacketId::ANGLE] = std::move(nextPkt);

    offset = 16;
    nextPkt = unique_ptr<Packet>(new ChargingState(data_ptr+offset));
    m_PktList[PacketId::CHARGE_STATE] = std::move(nextPkt);

    offset = 17;
    nextPkt = unique_ptr<Packet>(new Voltage(data_ptr+offset));
    m_PktList[PacketId::VOLTAGE] = std::move(nextPkt);

    offset = 19;
    nextPkt = unique_ptr<Packet>(new Current(data_ptr+offset));
    m_PktList[PacketId::CURRENT] = std::move(nextPkt);

    offset = 21;
    nextPkt = unique_ptr<Packet>(new Temperature(data_ptr+offset));
    m_PktList[PacketId::TEMPERATURE] = std::move(nextPkt);

    offset = 22;
    nextPkt = unique_ptr<Packet>(new BatteryCharge(data_ptr+offset));
    m_PktList[PacketId::CHARGE] = std::move(nextPkt);

    offset = 24;
    nextPkt = unique_ptr<Packet>(new BatteryCapacity(data_ptr+offset));
    m_PktList[PacketId::CAPACITY] = std::move(nextPkt);

    offset = 26;
    nextPkt = unique_ptr<Packet>(new WallSignal(data_ptr+offset));
    m_PktList[PacketId::WALL_SIGNAL] = std::move(nextPkt);

    offset = 28;
    nextPkt = unique_ptr<Packet>(new CliffLeftSignal(data_ptr+offset));
    m_PktList[PacketId::CLIFF_LEFT_SIGNAL] = std::move(nextPkt);

    offset = 30;
    nextPkt = unique_ptr<Packet>(new CliffFrontLeftSignal(data_ptr+offset));
    m_PktList[PacketId::CLIFF_FRONT_LEFT_SIGNAL] = std::move(nextPkt);

    offset = 32;
    nextPkt = unique_ptr<Packet>(new CliffFrontRightSignal(data_ptr+offset));
    m_PktList[PacketId::CLIFF_FRONT_RIGHT_SIGNAL] = std::move(nextPkt);

    offset = 34;
    nextPkt = unique_ptr<Packet>(new CliffRightSignal(data_ptr+offset));
    m_PktList[PacketId::CLIFF_RIGHT_SIGNAL] = std::move(nextPkt);

    //Offset Unused 2 --> 36  (1 Byte)
    // Offset Unused 3 --> 37 (2 bytes)
    offset = 39;
    nextPkt = unique_ptr<Packet>(new ChargingSourceAvailable(data_ptr+offset));
    m_PktList[PacketId::CHARGE_SOURCE] = std::move(nextPkt);

    offset = 40;
    nextPkt = unique_ptr<Packet>(new OIMode(data_ptr+offset));
    m_PktList[PacketId::OI_MODE] = std::move(nextPkt);

    // Offset Song Number --> 41 (1 byte)
    // Offset Song Playing --> 42 (1 byte)

    offset = 43;
    nextPkt = unique_ptr<Packet>(new OiStreamNumPkts(data_ptr+offset));
    m_PktList[PacketId::NUM_STREAM_PACKETS] = std::move(nextPkt);

    offset = 44;
    nextPkt = unique_ptr<Packet>(new VelocityRequested(data_ptr+offset));
    m_PktList[PacketId::VEL] = std::move(nextPkt);

    offset = 46;
    nextPkt = unique_ptr<Packet>(new RadiusRequested(data_ptr+offset));
    m_PktList[PacketId::RADIUS] = std::move(nextPkt);

    offset = 48;
    nextPkt = unique_ptr<Packet>(new RequestedRightVelocity(data_ptr+offset));
    m_PktList[PacketId::RIGHT_VEL] = std::move(nextPkt);

    offset = 50;
    nextPkt = unique_ptr<Packet>(new RequestedLeftVelocity(data_ptr+offset));
    m_PktList[PacketId::LEFT_VEL] = std::move(nextPkt);
}

std::string Group6Pkt::LogHeaderStr(){
    return "Distance Angle WallSignal CliffLeft CliffLeftFront CliffRightFront CliffRight Voltage Current ChargingSrc";
}

std::string Group6Pkt::LogDataStr(Group6Pkt* pkt){
    ostringstream ss;

    int16_t distance =
        reinterpret_cast<const DistanceTravelled*>(
        pkt->getPacket(PacketId::DISTANCE))->getValue();
    ss << distance << " ";

    int16_t angle =
        reinterpret_cast<const AngleTurned*>(
        pkt->getPacket(PacketId::ANGLE))->getValue();
    ss << angle << " ";

    uint16_t wall_signal =
        reinterpret_cast<const WallSignal*>(
        pkt->getPacket(PacketId::WALL_SIGNAL))->getValue();
    ss << wall_signal << " ";

    uint16_t cliff_left =
        reinterpret_cast<const CliffLeftSignal*>(
        pkt->getPacket(PacketId::CLIFF_LEFT_SIGNAL))->getValue();
    ss << cliff_left << " ";

    uint16_t cliff_front_left =
        reinterpret_cast<const CliffFrontLeftSignal*>(
        pkt->getPacket(PacketId::CLIFF_FRONT_LEFT_SIGNAL))->getValue();
    ss << cliff_front_left << " ";

    uint16_t cliff_front_right =
        reinterpret_cast<const CliffFrontRightSignal*>(
        pkt->getPacket(PacketId::CLIFF_FRONT_RIGHT_SIGNAL))->getValue();
    ss << cliff_front_right << " ";

    uint16_t cliff_right =
        reinterpret_cast<const CliffRightSignal*>(
        pkt->getPacket(PacketId::CLIFF_RIGHT_SIGNAL))->getValue();
    ss << cliff_right << " ";

    int16_t voltage =
        reinterpret_cast<const Voltage*>(
        pkt->getPacket(PacketId::VOLTAGE))->getValue();
    ss << voltage/1000.0 << " ";

    int16_t current =
        reinterpret_cast<const Current*>(
        pkt->getPacket(PacketId::CURRENT))->getValue();
    ss << current/1000.0 << " ";

    uint8_t charging_src =
        reinterpret_cast<const ChargingSourceAvailable*>(
        pkt->getPacket(PacketId::CHARGE_SOURCE))->getValue();
    ss << to_string(charging_src) << " ";

   return ss.str();

}

} // Sensor
} // Roomba
