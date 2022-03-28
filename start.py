import time

import Server.server as server
import perception.oak.oak_perception as perception
import perception.oak.mapping as mapping
import perception.oak.plotting.custom_plotting as my_plt
import planning.AStar as global_planning
import planning.pure_pursuit as local_planning


# Launch Server for communications with the boats
server.launch()

task = "test"
down_sample_rate = 3
param_dict = perception.accept_json(task)

mapped = None
for i, p_out in enumerate(perception.im_process(param_dict)):
    """
    perception outputs (p_out):
        "imgs" - dict of (disp_win_name, image) from specified processing operations
        "state" - dict of (boom_or_boat, pose_theta) for each boat
        "origins" - dict of raw tag data from AprilTag detection
        "ground" - 2D array of pixel_coords for the coordinate system of the world (planning space)
    """
    imgs, all_states, origins, ground_plane = p_out["imgs"], p_out["state"], p_out["origins"], p_out["ground"]
    if i == 0:
        t = time.time()
    if i % down_sample_rate != 0:
        if ground_plane is not None and origins is not None and mapped is None:
            print()
            # define mapping class
            mapper = mapping.OccMap(param_dict["mapping"]) # TODO: Combine functionality with ocean.py class
            mapped = mapper.find_occ(imgs["pingpong_color_threshed"], ground_plane)
            # define planner and pursuer nodes for left boat (labelled boom)
            #boom_global_planner = global_planning.AStar(left=True) # TODO: INTEGRATE
            #boom_path = boom_global_planner.plan()
            boom_local_planner = local_planning.PurePursuit(sim=False) # TODO: IMPLEMENT, INTEGRATE, AND THREAD
            # define planner and pursuer nodes for right boat (labelled boat)
            #boat_global_planner = global_planning.AStar(left=False) # TODO: INTEGRATE
            #boat_path = boatq_global_planner.plan()
            boat_local_planner = local_planning.PurePursuit(sim=False) # TODO: IMPLEMENT, INTEGRATE, AND THREAD
        else:
            # TODO: Test
            steering_comms = {}
            if "boom" in all_states.keys():
                boom_pose = all_states["boom"]
                boom_local_planner.find_curr_next(boom_pose)
                steering_comms["boom"] = boom_local_planner.pursue(boom_pose)
            if "boat" in all_states.keys():
                boat_pose = all_states["boat"]
                boat_local_planner.find_curr_next(boat_pose)
                steering_comms["boat"] = boat_local_planner.pursue(boat_pose)
            print("frame: ", i, ", fps: ", (i + 1)/(time.time() - t))
    else:
        print()
        # update the map and the path
        # TODO: Implement update map function in OccMap class
        # TODO: Implement update path function in AStar class