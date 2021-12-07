# OSPREY - Oil SPill REcoverY (OSPREY)

![front-page](https://user-images.githubusercontent.com/55633473/145086720-254725aa-71fb-4222-ad1a-e567e157e9bb.PNG)

We are ESE Interdepartmental Senior Design (ISD) Team BoomBoat, comprised of University of Pennsylvania Engineering Seniors:

  - Justin Duhamel - Testing and Controls
  - Jason Friedman - Mechatronics and Controls
  - Zachary Goldberg - Electrical Engineering and Perception
  - Andrew Garrett - Motion Planning and Perception
  - Adam Liang - Hardware and Business

We are a group of talented, hungry, and driven individuals looking to change the world with robotics.  Our project BoomBoat sits at the intersection of engineering and sustainability practice.  Still hot in the process of development, we envision BoomBoat as a solution to the time and monetary inefficiencies of oil spill containment.

Current oil spill remediation techniques are costly, inefficient, and antiquated. Oil spill booms are the most popular method but lack the autonomy to be proactive, scalable, and time/cost efficient. As a result, oil/gas firms, remediation suppliers, and governmental agencies must discover new ways to revolutionize age-old containment solutions.  BoomBoat is a multi-robot, autonomous system inspired by the current state of the art in oil spill boom containment methods.  Shown below is a simplified explanation of how BoomBoat works:

![outline](https://user-images.githubusercontent.com/55633473/145088809-ba49e33f-6f59-43f9-a562-a5d6cff938c2.PNG)

Using two wifi-controlled boats and a overhead camera, BoomBoat can encompass arbitrarily shaped oil spills.  Thanks to careful selection and proper hardware sizing, we have settled on a hardware design that meets our scale limitations.  This is shown below.

![hardware](https://user-images.githubusercontent.com/55633473/145089282-71507dac-b35e-411d-9b8a-48a59c0b5d97.PNG)

Given that Boomboat is an autonomous system, performance is tied to how well our robot can understand its surroundings and navigate unfamiliar and dynamic environments.  At the heart of our autonomy platform are the perception and motion-planning modules.  We've made significant progress in experimenting with different methods for each of these subsystems.  See below for a high-level understanding of BoomBoat's proposed flow of information.

![system](https://user-images.githubusercontent.com/55633473/145094950-e68c5cf3-6d3f-4d99-98ee-2d17555347ae.PNG)

# Progress to Date:
 - # Perception:
    - Use MATLAB Color Thresholding for oil spill detection and data understanding (MATLAB)
    - Stream GoPro frames over TCP (GoPro Python API)
    - Perform Canny Edge detection at 20+ fps on GoPro stream (Numpy, OpenCV)
    - Extract Contours from Canny Edge at ~10fps on GoPro stream (Numpy, OpenCV)
    - Train and Tune Mask-RCNN for oil spill segmentation, inf. time: ~5fps (Numpy, Pytorch, Weights and Biases)
    - Calibrate GoPro Fisheye Lens (MATLAB, OpenCV)
    - Use AprilTag Detection for mapping planar surface (Numpy, pupil-labs)
 - # Motion Planning:
    - Use A* Search to compute shortest path for single boat around simulated oil spill gridworld (Numpy)
    - Experiment with different heuristics and methods of enforcing oil spill avoidance (Numpy)
 - # Hardware:
      - Determine approximate power requirements and select optimal hobby boat hardware for considerations of scale, budget, and quality
      - Order parts

# Up Next:
 - # Perception:
    - Characterize individual function runtimes for current GoPro perception module
    - Implement Segmentation Model on GoPro and tweak for faster running time
    - Localize boats in 2D planar projection
 - # Motion Planning:
    - Develop measure for tradeoff between oil spill avoidance and path cost
    - Explore topological constraints of environment for advanced A*
    - Implement two-boat path-planning
 - # Hardware:
    - Bench test electronic and mechanical components (motor, ESC, wifi controller, servos, etc.)
    - Waterproof interior (epoxy)
    - Test System-level functionality on dry-land
    - Implement PID controller for motion-planning motor commands
 - # Full System Integration:
    - Integrate XBox Controller wifi based individual boat guidance
    - Tune PID controller to account for ocean currents
    - Use simulated input for planning and guiding boats around a potential oil spill (vision used for validation)
    - Test Vision system on fake oil-spill identification (likely to be a tarp or confetti)
    - Use perception module outputs for safe navigation around fake oil-spill
 
# Thanks
We'd like to thank Penn Engineering professors Dr. Ani Hsieh and Dr. Pratik Chaudhari for serving as professional advisors to Team BoomBoat.  We'd also like to thank Dr. Sid Deliwala and Dr. Jan Van der Spiegel for answering any questions we ever have, be it at 2am or 2pm.
