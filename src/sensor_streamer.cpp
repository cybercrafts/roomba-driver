#include "sensor_streamer.h"
#include <iostream>
#include <vector>
#include <cctype>
#include <sstream>
#include <strings.h>

using namespace std;

namespace Roomba {
//------------------------------------------------------------------------------
// SensorStreamer
//------------------------------------------------------------------------------

SensorStreamer::SensorStreamer(SerialPort* serial_port,
    const std::vector<Roomba::Sensor::PacketId>& pkt_list)
    : m_SerialPort(serial_port)
{
    // for (int i = 0; i < pkt_list.size(); i++){
    //     m_pkt_list.push_back(std::move(pkt_list[i]));
    // }
    // m_RxBuffer.resize();
}

//------------------------------------------------------------------------------
// ~SensorStreamer
//------------------------------------------------------------------------------

SensorStreamer::~SensorStreamer(){
}

//------------------------------------------------------------------------------
// start
//------------------------------------------------------------------------------

bool
SensorStreamer::start(){

}

//------------------------------------------------------------------------------
// stop
//------------------------------------------------------------------------------

bool
SensorStreamer::stop(){

}

} // namespace Roomba