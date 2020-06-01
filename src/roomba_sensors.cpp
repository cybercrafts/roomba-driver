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

    uint8_t* data_ptr = getDataPtr();

    int offset = 0;
    unique_ptr<Packet> nextPkt(new BumpAndWheelDrop(data_ptr+offset));
    m_PktList[PacketId::BUMP_WHEELDROP] = std::move(nextPkt);

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

}

} // Sensor
} // Roomba
