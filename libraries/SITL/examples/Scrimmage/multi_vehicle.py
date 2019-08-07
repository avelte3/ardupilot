#!/usr/bin/env python3

import os
import subprocess
import signal
import sys
import argparse
import json
import math
from jinja_missions import mission_gen

 
def sigint_handler(signum, frame):
    subprocess.Popen('pkill -f "arducopter"', shell=True)
    subprocess.Popen('pkill -f "scrimmage"', shell=True)
    subprocess.Popen('pkill -9 -f "mavproxy.py"', shell=True)
    sys.exit(0)

def launch_ardupilot(vehicle_dict, origin, pattern):
    start = 0
    i = 0
    miss_config = {}
    wd = os.getcwd()
    for vehicle_name in vehicle_dict:
        for i in range(start, start+vehicle_dict[vehicle_name]["instances"]):
            dir_string = "{0}/swarm/{1}{2}".format(wd, vehicle_name, i)
            os.makedirs(dir_string, exist_ok=True)
            loc = pattern_generator(pattern, origin, i)
            cmd_string = 'xterm +hold -T {0}{1} -e "sim_vehicle.py -v {0} -I{1} -f {2} -l {3}"'.format(vehicle_name, i, vehicle_dict[vehicle_name]["frame"], str(loc))
            print(cmd_string)
            subprocess.Popen(cmd_string, shell=True, cwd=dir_string)
            miss_config[i] = {"vehicle": vehicle_name, "lat":loc.lat, "lon":loc.lon, "altitude":loc.alt, "heading":loc.head}
        start = i+1
        os.chdir(wd)
    
    print(miss_config)
    return miss_config

def start_scrimmage(mission_file):
    cmd_string = 'xterm -hold -T SCRIMMAGE -e "scrimmage {0}"'.format(mission_file)
    subprocess.Popen(cmd_string, shell=True)

def pattern_generator(pattern, origin, index):
    if pattern["name"] == 'line':
        R = 6378.1 #Radius of the Earth in km
        brng = math.radians(origin.head) #Bearing is 90 degrees converted to radians.
        d = pattern["separation"]/1000.0 * index #Distance in km from 

        lat_o = math.radians(origin.lat) #Current lat point converted to radians

        lat2 = math.asin( math.sin(lat_o)*math.cos(d/R) +
            math.cos(lat_o)*math.sin(d/R))

        lat2 = math.degrees(lat2)
        return LatLonAltHead("{0},{1},{2},{3}".format(lat2, origin.lon, origin.alt, origin.head))
    else:
        print("invalid pattern name")

class LatLonAltHead:
    def __init__ (self, string):
        self.lat = float(string.split(",")[0])
        self.lon = float(string.split(",")[1])
        self.alt = float(string.split(",")[2])
        self.head = float(string.split(",")[3])

    def __str__(self):
        return "{0},{1},{2},{3}".format(self.lat, self.lon, self.alt, self.head)

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)
signal.signal(signal.SIGHUP, sigint_handler)

parser = argparse.ArgumentParser(description='Multiple sim_vehicle launcher')
opt = parser._action_groups.pop()
req = parser.add_argument_group('required arguments')
req.add_argument('-i', '--instances', type = int, help='Number of instances to be launched')
req.add_argument('-f', '--frame', help='scrimmage-plane or scrimmage-copter')
req.add_argument('-v', '--vehicle', help='Either ArduCopter or Arduplane')
opt.add_argument('-j', '--json', help='JSON file to be used instead of command line arguments. Removes requirement of other arguments')
opt.add_argument('-p', '--pattern', default='line', choices=['line'], help='Pattern used to generate multiple vehicles.')
opt.add_argument('-o', '--origin', default='34.458281,-84.180209,450,130', help='set custom origin location (lat,lon,alt,heading)')
opt.add_argument('-s', '--separation', default='1.0', help='separation between vehicles in pattern in meters')
parser._action_groups.append(opt)
args = parser.parse_args()

processlist = []

if args.json == None:
    print("No JSON")
    config = {args.vehicle: {"instances":args.instances,"frame":args.frame}}
    origin = LatLonAltHead(args.origin)
    pattern = {"name":args.pattern, "separation":args.separation}
    miss_config = launch_ardupilot(config, origin, pattern)
else:
    print("Reading from JSON:", args.json)
    try:
        with open(args.json) as config_file:    
            config_dict = json.load(config_file)
        miss_config = launch_ardupilot(config_dict["vehicles"], LatLonAltHead(config_dict["origin"]), config_dict["pattern"])
    except FileNotFoundError:
        print(args.json, ": File does not exist.")
        sys.exit(1)

mission_gen(miss_config, 'miss_base.xml')
start_scrimmage('arducopter_gen.xml')


while True:
    pass
