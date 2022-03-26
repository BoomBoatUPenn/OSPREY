import json

ROOT = ".\\perception\\oak\\"

# for pottruck testing
test_config = {
    "task": "test",
    "undistort": True,
    "april": True,
    "pingpong_color_threshed": True,
    "display": False,
    "mapping": {"distance": 2,
                "density": 0.3}
    }

# for debugging
debug_config = {
    "task": "debug",
    "undistort": True,
    "april": True,
    "pingpong_color_threshed": True,
    "display": True,
    "mapping": {"distance": 2,
                "density": 0.3}
    }

if __name__ == "__main__":

    with open(ROOT + 'configs\\debug_config.json', 'w') as json_file:
        json.dump(debug_config, json_file)
    with open(ROOT + 'configs\\test_config.json', 'w') as json_file:
        json.dump(test_config, json_file)