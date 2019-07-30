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
#include <iostream>
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
    if (strcmp(frame_str,"scrimmage-copter")== 0) {
        mission_name_ = "arducopter.xml";
        motion_model_ = "Multirotor";
        visual_model_ = "iris";
    }
}

void Scrimmage::set_config(const char *config)
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
    char *end_str;
    char* copy_config = strdup(config);
    char *token = strtok_r(copy_config, ",", &end_str);
    while (token != NULL)
    {
        char *end_token;
        char *token2 = strtok_r(token, "=", &end_token);
        if (strcmp(token2, "mission")==0){
            mission_name_ = strtok_r(NULL, "=", &end_token);
            printf("saved mission: %s",mission_name_);
        } else if (strcmp(token2, "motion_model")==0){
            motion_model_ = strtok_r(NULL, "=", &end_token);
            printf("saved motion model: %s",motion_model_);
        } else if (strcmp(token2, "visual_model")==0){
            visual_model_ = strtok_r(NULL, "=", &end_token);
            printf("saved visual model: %s",visual_model_);
        } else if (strcmp(token2, "terrain")==0){
            terrain_ = strtok_r(NULL, "=", &end_token);
            printf("saved terrain: %s",terrain_);
        } else {
            printf("Invalid scrimmage param: %s", token2);
        }
        printf("\n");
        token = strtok_r(NULL, ",", &end_str);
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
        char full_exec_str[512] = "";
        sprintf(full_exec_str, "xterm +hold -T SCRIMMAGE -e 'scrimmage %s -o \"latitude_origin=%f,longitude_origin=%f,altitude_origin=%f,heading=%f,motion_model=%s,visual_model=%s,terrain=%s,to_ardupilot_port=%d,from_ardupilot_port=%d\"'",mission_name_, home.lat*1.0e-7f, home.lng*1.0e-7f, home.alt*1.0e-2f, home_yaw, motion_model_, visual_model_, terrain_,fdm_port+1,fdm_port);

        printf("%s\n", full_exec_str);

        // system call worked
        int ret = system(full_exec_str);

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
