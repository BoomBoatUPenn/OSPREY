import time

import Server.server as server
import perception.oak.oak_perception as perception
import perception.oak.mapping as mapping
import perception.oak.plotting.custom_plotting as my_plt
import planning.AStar as global_planning
import planning.pure_pursuit as local_planning


# Launch Server for communications with the boats
server.launch()

task = "debug"
down_sample_rate = 5
param_dict = perception.accept_json(task)

mapped = None
boom_local_planner = None
boat_local_planner = None
server.Autonomous = True

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
        print("System Initialized")
    if i % down_sample_rate != 0:
        if (ground_plane is not None and origins is not None) and mapped is None:
            print("Creating Map, Localizing Boats, and Planning Path")
            # define mapping class
            mapper = mapping.OccMap(param_dict["mapping"]) # TODO: Combine functionality with ocean.py class
            mapped = mapper.find_occ(imgs[0]["pingpong_color_threshed"], ground_plane)
            # define planner and pursuer nodes for left boat (labelled boom)
            #boom_global_planner = global_planning.AStar(left=True) # TODO: INTEGRATE
            #boom_path = boom_global_planner.plan()
            boom_local_planner = local_planning.PurePursuit(sim=False) # TODO: IMPLEMENT, INTEGRATE, AND THREAD
            # define planner and pursuer nodes for right boat (labelled boat)
            #boat_global_planner = global_planning.AStar(left=False) # TODO: INTEGRATE
            #boat_path = boat_global_planner.plan()
            boat_local_planner = local_planning.PurePursuit(sim=False) # TODO: IMPLEMENT, INTEGRATE, AND THREAD
            # TODO: Test
        if "boom" in all_states.keys() and boom_local_planner is not None:
            boom_pose = all_states["boom"]
            print("Boom Pose: ", boom_pose)
            if boom_pose is not None:
                boom_local_planner.find_curr_next(boom_pose[0])
                boom_comms = boom_local_planner.pursue(boom_pose)
                server.Boat1.RudderInput = boom_comms[0]
                print("new rudder input {}".format(server.Boat1.RudderInput))
                server.Boat1.Throttle = boom_comms[1]
        if "boat" in all_states.keys() and boat_local_planner is not None:
            boat_pose = all_states["boat"]
            print("Boat Pose: ", boat_pose)
            if boat_pose is not None:
                boat_local_planner.find_curr_next(boat_pose[0])
                boat_comms = boat_local_planner.pursue(boat_pose)
                server.Boat2.RudderInput = boat_comms[0]
                server.Boat2.Throttle = boat_comms[1]

        print("frame: ", i, ", fps: ", (i + 1)/(time.time() - t))
    else:
        print()
        # update the map and the path
        # TODO: Implement update map function in OccMap class
        # TODO: Implement update path function in AStar class