#include <memory>
#include <cstdint>
#include <vector>
#include "roomba_open_interface.h"

class RoombaController {
public:
    static std::unique_ptr<RoombaController> NewInstance(
        const std::string& usb_port
    );
    RoombaController() = delete;
    ~RoombaController();

    bool initialize();
    void terminate();
    Roomba::OIMode getCurrentOIMode();
    bool toSafeMode(){
        return changeOIMode(Roomba::OIMode::SAFE);
    }
    bool toFullMode(){
        return changeOIMode(Roomba::OIMode::FULL);
    }

    uint16_t getRightEncoder();
    void reset();

    bool drive(int16_t vel_in_mm_sec);

private:
    explicit RoombaController(int fd);
    static std::string ToString(const std::vector<uint8_t>& data);

    static bool ConfigureSerial(int fd);

    bool startOI();
    void stopOI();
    void powerDown();
    bool changeOIMode(Roomba::OIMode desired_mode);
    void intTo2sComplementBytes(int16_t int_val, uint8_t bytes[2]);

    const int   m_fd{0};
    bool        m_initialized{false};
};