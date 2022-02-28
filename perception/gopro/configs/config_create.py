import json

test_config = {
    "task": "testing",
    "april": True,
    "edges": False,
    "contours": False,
    "contour_thresh": 30,
    "pingpong_color_threshed": False,
    "undistort": True,
    "record": False,
    "boat_color_threshed": False
    }

with open('configs\\test_config.json', 'w') as json_file:
    json.dump(test_config, json_file)