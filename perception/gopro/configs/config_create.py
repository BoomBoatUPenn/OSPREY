import json

ROOT = "C:\\Schoolwork\\2022 Spring Semester\\Senior Design\\OSPREY_dev\\perception\\gopro\\"

test_config = {
    "task": "debugging",
    "april": True,
    "edges": False,
    "contours": False,
    "contour_thresh": 30,
    "pingpong_color_threshed": False,
    "undistort": True,
    "record": False,
    "boat_color_threshed": False,
    "display": True
    }

with open(ROOT + 'configs\\debug_config.json', 'w') as json_file:
    json.dump(test_config, json_file)