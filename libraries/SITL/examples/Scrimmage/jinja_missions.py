#!/usr/bin/env python3

import os
from jinja2 import Environment, FileSystemLoader, select_autoescape

def mission_gen(entities, template):
    env = Environment(
        loader=FileSystemLoader(os.path.join(os.path.dirname(__file__), 'templates')),
        autoescape=select_autoescape(['xml'])
    )

    miss_temp = env.get_template(template)

    output = miss_temp.render(dic=entities)
    # print(output)
    with open("arducopter_gen.xml", "w") as fh:
        fh.write(output)

if __name__== "__main__":
    dic = {0:{"x":0.0, "y":0.0, "z":0.0, "vehicle":"ArduCopter"},
        1:{"x":1.0, "y":0.0, "z":0.0, "vehicle":"ArduCopter"},
        2:{"x":2.0, "y":0.0, "z":0.0, "vehicle":"ArduCopter"},
        3:{"x":3.0, "y":2.0, "z":0.0, "vehicle":"ArduCopter"}
    }

    # dic = {"ArduCopter":{"frame": "scrimmage-copter",
    #                       "instances": 2,
    #                       "x":0.0, "y":0.0, "z":0.0},
    #         "ArduPlane":{"frame": "scrimmage-plane",
    #                      "instances" : 2,
    #                       "x":10.0, "y":10.0, "z":0.0}}

    mission_gen(dic, 'miss_base.xml')