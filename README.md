    # turtle_fusion
This is the repository developed for the LiDAR-Camera fusion implementation for the racing season 2021-2022

    ## Introduction 
By the term *fusion* or *LiDAR-Camera fusion* we refer to the merging of data given by both sensors and the production of new data that can be used for extracting more accurate or more reliable results. For our problem we want to be able to detect the designated cones that delinate the track and accuratley measure their distances from our vehicle. The more correct the observations are, the better our mapping will be and thus the vehicle can navigate the track much easier without the fear of knocking down cones. 

    ## Theory
The main idea behind this process is to get the best of both worlds from the perception sensors. We exploit the camera's capability to recognize the cones in conjuction with the LiDAR'S ability to measure object distances at a high level accuracy.  

    ## Branches 
Each branch of this repository is a different working implementation of the fusion algorithm. We have the `master` branch, the `pure_function_logic` and `multi_node`

    ## master
This was the first developing branch so it can be described as the concrete base of the whole repository. It holds the main structure of the algorithm plus the necessary files in order for the code to work. 

It's not the prefferable repository to run on the vehicle rather than it can be used as {legacy code} for it is the basis of the repo. 

    ## pure_function_logic
The main difference between this branch and `master` is that in *master* everything is a class and functions are class instances. Here -as the name proposes- we call pure functions that run in orded called by a {main} without any class instance. Both `pure_function_logic` branch and `master` suffered from thread-racing-conditions that we had to tackle in order to make our code thread-safe.

    ## multi-node
As the name describes, `multi_node` is the branch in which we make use of multiple nodes - one for each camera - just to manage the multithreading between processes, plus the (pseudo)asynchronous nature of our camera pipeline made very difficult to come up against timing issues between all four perception sensors. Considering these problems, it made perfect sense to isolate each camera to a single node, as ROS2 can manage by itself the real time multinode execution. 

As our testings shown us, this is the preferable logic to be used in our vehicle, compared to the other two, as it is faster and more reliable to extract results.

    ## Run the code 
As the code is a ROS2 executable, on your terminal type `ros2 run turtle_fusion camera_lidar_fusion` - after checking that the working branch is the {multi_node}. By running you can see some {ROS_INFO} messages indicating that all nodes initialized and are ready to read LiDAR and Camera messages.
