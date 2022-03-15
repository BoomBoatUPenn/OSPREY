from email.policy import default
import json
import argparse

ROOT = "C:\\Schoolwork\\2022 Spring Semester\\Senior Design\\OSPREY_dev\\perception\\gopro\\"

# for pottruck testing
test_config = {
    "task": "testing",
    "april": True,
    "edges": False,
    "contours": False,
    "contour_thresh": 30,
    "pingpong_color_threshed": True,
    "undistort": True,
    "record": True,
    "boat_color_threshed": False,
    "display": False
    }

# for debugging
debug_config = {
    "task": "debugging",
    "april": True,
    "edges": False,
    "contours": False,
    "contour_thresh": 30,
    "pingpong_color_threshed": True,
    "undistort": True,
    "record": True,
    "boat_color_threshed": False,
    "display": True
    }

if __name__ == "__main__":
    argv = argparse.ArgumentParser()
    argv.add_argument(name="--debug", default=True, type=bool, required=False)
    argv.parse_args()
    
    if argv[0]:
        with open(ROOT + 'configs\\debug_config.json', 'w') as json_file:
            json.dump(debug_config, json_file)
    else:
        with open(ROOT + 'configs\\test_config.json', 'w') as json_file:
            json.dump(test_config, json_file)