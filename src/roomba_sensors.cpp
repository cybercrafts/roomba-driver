#include "roomba_sensors.h"

namespace Roomba{
namespace Sensor{

std::unique_ptr<Packet>
CreateSensorPacket(PacketId id, uint8_t* data_ptr){
    switch (id){
        case PacketId::OI_MODE:
            return std::unique_ptr<Packet>(new OIMode(data_ptr));
        break;
        case PacketId::DISTANCE:
            return std::unique_ptr<Packet>(new DistanceTravelled(data_ptr));
        break;
        case PacketId::ANGLE:
            return std::unique_ptr<Packet>(new AngleTurned(data_ptr));
        break;
        case PacketId::VOLTAGE:
            return std::unique_ptr<Packet>(new Voltage(data_ptr));
        break;
        default:
        return nullptr;
    }
}


} // Sensor
} // Roomba
