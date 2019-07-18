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
  simulator connector for Scrimmage simulator
*/

#include "SIM_Scrimmage.h"

#include <fcntl.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Scrimmage::Scrimmage(const char *home_str, const char *_frame_str) :
    Aircraft(home_str, _frame_str),
    prev_timestamp_us(0),
    recv_sock(true),
    send_sock(true),
    frame_str(_frame_str)
{
    // Set defaults for scrimmage-copter
    if (std::string(frame_str) == "scrimmage-copter") {
        mission_name_ = "arducopter.xml";
        motion_model_ = "Multirotor";
        visual_model_ = "iris";
    }
}

void Scrimmage::set_config(const std::string &config)
{
    fdm_port = 5504 + instance*10;
    printf("ArduPilot sending to scrimmage on port: %d\n", fdm_port);
    printf("ArduPilot listening to scrimmage on port: %d\n", fdm_port+1);

    // TODO: un-hard code IP address and allow port to be changed.
    recv_sock.bind("127.0.0.1", fdm_port+1);

    recv_sock.reuseaddress();
    recv_sock.set_blocking(false);

    send_sock.reuseaddress();
    send_sock.set_blocking(false);

    // The configuration string is a comma-separated sequence of key:value
    // pairs

    // First iterate over comma-separated strings
    std::stringstream comma_ss(config);
    std::string comma_str;
    while (getline(comma_ss, comma_str, ',')) {
        // Parse the key=value strings
        std::vector<std::string> tokens;
        std::stringstream kv_ss(comma_str);
        std::string intermediate;
        while (getline(kv_ss, intermediate, '=')) {
            tokens.push_back(intermediate);
        }

        // If the key:value was parsed correctly, add it to the config map
        if (tokens.size() == 2) {
            configs_[tokens[0]] = tokens[1];
        }
    }

    // Were any of the defaults overwritten?
    auto it_mission = configs_.find("mission");
    if (it_mission != configs_.end()) {
        mission_name_ = it_mission->second;
    }

    auto it_motion = configs_.find("motion_model");
    if (it_motion != configs_.end()) {
        motion_model_ = it_motion->second;
    }

    auto it_visual = configs_.find("visual_model");
    if (it_visual != configs_.end()) {
        visual_model_ = it_visual->second;
    }

    auto it_terrain = configs_.find("terrain");
    if (it_terrain != configs_.end()) {
        terrain_ = it_terrain->second;
    }

    // start scrimmage after parsing simulation configuration
    start_scrimmage();
}

// Start Scrimmage child
void Scrimmage::start_scrimmage(void)
{
    pid_t child_pid = fork();
    if (child_pid == 0) {
        // In child

        // Construct the scrimmage command string with overrides for initial
        // position and heading.
        std::string full_exec_str = "'scrimmage " + mission_name_
            + " -o \""
            + "latitude_origin=" + std::to_string(home.lat / 1.0e7f) + ","
            + "longitude_origin=" + std::to_string(home.lng / 1.0e7f) + ","
            + "altitude_origin=" + std::to_string(home.alt / 1.0e2f) + ","
            + "heading=" + std::to_string(home_yaw) + ","
            + "motion_model=" + motion_model_ + ","
            + "visual_model=" + visual_model_ + ","
            + "terrain=" + terrain_ + ","
            + "to_ardupilot_port=" + std::to_string(fdm_port+1) + ","
            + "from_ardupilot_port=" + std::to_string(fdm_port)
            + "\"'";

        std::cout << full_exec_str.c_str() << std::endl;

        // system call worked
        full_exec_str = "xterm +hold -T SCRIMMAGE -e " + full_exec_str;
        int ret = system(full_exec_str.c_str());

        if (ret != 0) {
            std::cerr << "scrimmage didn't open.\n";
            perror("scrimmage");
        }
    }
}

/*2
Serial port 2 on TC
  send servos
*/
void Scrimmage::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;

    for (int i = 0; i < MAX_NUM_SERVOS; i++) {
        pkt.servos[i] = input.servos[i];
    }
    send_sock.sendto(&pkt, sizeof(servo_packet), "127.0.0.1", fdm_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Scrimmage::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    // Re-send the servo packet every 0.1 seconds until we get a reply. This
    // allows us to cope with some packet loss to the FDM
    while (recv_sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        //send_servos(input);
    }

    // auto-adjust to simulation frame rate
    uint64_t dt_us = 0;
    if (pkt.timestamp_us > prev_timestamp_us)
        dt_us = pkt.timestamp_us - prev_timestamp_us;
    time_now_us += dt_us;

    double dt = ((double)dt_us) / 1.0e6;
    if (dt < 0.01 && dt > 0) {
        adjust_frame_time(1.0/dt);
    }
    prev_timestamp_us = pkt.timestamp_us;

    // dcm_bl: dcm from body to local frame
    dcm.from_euler(pkt.roll, pkt.pitch, pkt.yaw);
    dcm.normalize();

    // subtract gravity to get specific force measuremnt of the IMU
    accel_body = Vector3f(pkt.xAccel, pkt.yAccel, pkt.zAccel) - dcm.transposed()*Vector3f(0.0f, 0.0f, GRAVITY_MSS);
    gyro = Vector3f(pkt.rollRate, pkt.pitchRate, pkt.yawRate);

    ang_accel = (gyro - gyro_prev) / std::max(.000001, dt);
    gyro_prev = gyro;

    velocity_ef = Vector3f(pkt.speedN, pkt.speedE, pkt.speedD);

    location.lat = pkt.latitude * 1.0e7;
    location.lng = pkt.longitude * 1.0e7;
    location.alt = pkt.altitude * 1.0e2;
    position.z = (home.alt - location.alt) * 1.0e-2;


    // velocity relative to air mass, in earth frame TODO
    velocity_air_ef = velocity_ef;

    // velocity relative to airmass in body frame TODO
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    battery_voltage = 0;
    battery_current = 0;
    rpm1 = 0;
    rpm2 = 0;

    airspeed = pkt.airspeed;
    airspeed_pitot = pkt.airspeed;
}

/*
  update the Scrimmage simulation by one time step
 */
void Scrimmage::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_mag_field_bf();
}

} // namespace SITL
