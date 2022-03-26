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
down_sample_rate = 9
param_dict = perception.accept_json(task)
for i, p_out in enumerate(perception.im_process(param_dict)):
    """
    perception outputs (p_out):
        "imgs" - dict of (disp_win_name, image) from specified processing operations
        "state" - dict of (boom_or_boat, pose_theta) for each boat
        "origins" - dict of raw tag data from AprilTag detection
        "ground" - 2D array of pixel_coords for the coordinate system of the world (planning space)
    """
    imgs, all_states, origins, ground_plane = p_out["imgs"], p_out["state"], p_out["origins"], p_out["ground"]
    if i % down_sample_rate != 0 or i == 0:
        if i == 0:
            t = time.time()
            # create initial map
            mapper = mapping.OccMap(param_dict["mapping"]) # TODO: Combine functionality with ocean.py class
            map = mapper.find_occ(imgs["pingpong_color_threshed"], ground_plane)
            # define planner and pursuer nodes for left boat (labelled boom)
            boom_global_planner = global_planning.AStar(left=True) # TODO: INTEGRATE
            boom_path = boom_global_planner.plan()
            boom_local_planner = local_planning.PurePursuit() # TODO: IMPLEMENT, INTEGRATE, AND THREAD
            # define planner and pursuer nodes for right boat (labelled boat)
            boat_global_planner = global_planning.AStar(left=False) # TODO: INTEGRATE
            boat_path = boat_global_planner.plan()
            boat_local_planner = local_planning.PurePursuit() # TODO: IMPLEMENT, INTEGRATE, AND THREAD
        
        # TODO: Implement Path following in PurePursuit class
    else:
        print()
        # update the map and the path
        # TODO: Implement update map function in OccMap class
        # TODO: Implement update path function in AStar class
    
    print("frame: ", i, ", fps: ", (i + 1)/(time.time() - t))
    # my_plt.plot_Occ(ground_plane, map, param_dict["mapping"])