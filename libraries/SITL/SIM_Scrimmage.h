/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connection for Scrimmage Simulator
*/

#pragma once

#include <string>
#include <map>

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a last_letter simulator
 */
class Scrimmage : public Aircraft {
public:
    Scrimmage(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Scrimmage(home_str, frame_str);
    }

    void set_config(const std::string &config) override;

private:
    uint16_t fdm_port;

    /*
      packet sent to Scrimmage from ArduPilot
     */
    static const int MAX_NUM_SERVOS = 16;
    struct servo_packet {
        uint16_t servos[MAX_NUM_SERVOS];
    };

    /*
      state packet sent from Scrimmage to ArduPilot
     */
    struct fdm_packet {
        uint64_t timestamp_us; // simulation time in microseconds
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void start_scrimmage(void);

    uint64_t prev_timestamp_us;
    SocketAPM recv_sock;
    SocketAPM send_sock;

    std::string frame_str;
    std::map<std::string, std::string> configs_;

    double home_yaw_deg_ = 0;

    // Use ArduPlane by default
    std::string mission_name_ = "arduplane.xml";
    std::string motion_model_ = "JSBSimControl";
    std::string visual_model_ = "zephyr-blue";
    std::string terrain_ = "mcmillan";
};

} // namespace SITL
