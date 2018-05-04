# MDB

I am going to merge everything related to MDB here... Work in progress...

## mdb_ltm

This is the third leg of the Multilevel Darwinist Brain (MDB). This one, the Motivational Engine (https://github.com/robotsthatdream/mdb_motiven), and the STM component (https://github.com/robotsthatdream/mdb_cafer) should be fully integrated with CAFER at the end of this project.

The objective of this component is to play in Long-Term Memory with the necessary elements to:

- Facilitate the recall of previously acquired knowledge during the operation of a robot / agent whenever it can be useful, using a filter mechanism that pre-activates those things that were successfully jointly activated in the past in similar, but not necessarily equal, circumstances. If the recalled knowledge is also useful in the new context, that information is added to that mechanism (so, the system generalizes).
- Make possible dreaming.

Please, take into account that this is a "work-in-progress" software.

There are two very different branches:

- The *preros* branch includes a software prototype (*ugly*, quickly developed, with no software engineering principles in mind) used to test ideas for the DREAM 3rd Hackademia and to obtain results for the IWINAC 2017.
- The *ros* branch is a ROS (http://www.ros.org) project and it is the current default branch. The objective of this branch is to write a proper LTM software component for using it with MDB and CAFER, so it uses ROS (and CAFER later on) to work with a real robot or a ROS compatible simulator as Gazebo. As in the *preros* branch, everything is written in Python. Its development began using ROS Indigo, but now that the Baxter Simulator (https://github.com/RethinkRobotics/baxter_simulator.git) is Kinetic ready, we will start using ROS Kinetic soon.

### MDB inter-component communication specification

While a complete communication mechanism between components in the DREAM project is not ready, we are using a set of ROS topics, messages and services to exchange information between components of the MDB, that could be used also for other people at the DREAM project.

The idea is that when some component generates a new piece of knowledge (a policy, a forward model, etc.), we should support three different ways of executing / use that new knowledge nugget:
- Remote execution. This is useful when the data format is not supported for the component that needs that knowledge nugget. The knowledge generator publishes in a given topic the availability of the new knowledge nugget, as well as a service address to remotely run that knowledge nugget, so every component can call that service to obtain a result for a given input. Obviously, it is the least efficient way of using it, as implies a communication everytime that the knowledge nugget is needed.
- Get the data. This is useful when the data format is supported for the component that wants to use the knowledge nugget. The knowledge generator publishes in a given topic the availability of the new knowledge nugget, as well as a service address to obtain the data that compose the knowledge nugget, so whenever a request is received in that service, the knowledge nugget is sent to the component that requests it so. This is very efficient, as it only implies a communication the first time, but both ends of the communication should understand the data. Right now, this data format agreement is done "by hand" by the programmers, but we should use some kind of data directory and, probably, an ontology.
- Get the code. This is useful when the code that can run the knowledge nugget is accesible from the component that wants to use it. This code can be a class able to understand and run the knowledge nugget, or it can be the knowledge nugget itself (for instance, a hand-made policy). So, the knowledge generator publishes in a given topic the availability of the new knowledge nugget, as well as the class name to execute that knowledge nugget. This is not only efficient, but also very convenient, as you can expose new classes (and, therefore, knowledge interpreters) in runtime to any ROS node (in our case, the LTM) without stopping it. You need to implement some kind of dynamic class loading (which is very easy in Python, by the way).

Therefore, those messages published in the ROS topics to advertise new knowledge nuggets should have the following fields at least:
- Command. Right now, we only need a *new* command. But in the LTM a *delete* command will be necessary too. Other commands such as *replace* could be convenient.
- Id.
- ROS service address to execute a knowledge nugget.
- ROS service address to obtain a knowledge nugget representation / data.
- Class name to execute the knowledge nugget. In Python: package.module.class.
- Language in which the knowledge nugget is implemented: *python*, *cpp*, *java*...
Command and id cann't be null. At least one of the fields related to execution / obtaining of a knowledge nugget needs to be not null.
Right now, every field is a string to make easier the debugging of messages.

This are the ROS topic names (and associated ROS messages) that we are using to publish those announcement messages. We are using ROS parameteres and dynamic class loading so, actually, their names are irrelevant:
- /mdb/p_node (mdb_ltm.msg.PNodeMsg)
- /mdb/goal (mdb_ltm.msg.GoalMsg)
- /mdb/value_function (mdb_ltm.msg.ValueFunctionMsg)
- /mdb/forward_model (mdb_ltm.msg.ForwardModelMsg)
- /mdb/policy (mdb_ltm.msg.PolicyMsg)
- /mdb/c_node (mdb_ltm.msg.CNodeMsg)

These are MDB tentative ROS service names for execution / getting fields:
- /mdb_stm/execute_policy
- /mdb_stm/get_policy
- /mdb_stm/execute_forward_model
- /mdb_stm/get_forward_model
- /mdb_stm/forward_models_server
- /mdb_motiven/execute_goal
- /mdb_motiven/get_goal
- /mdb_motiven/execute_value_function
- /mdb_motiven/get_value_function
- /mdb_motiven/execute_perceptual_class
- /mdb_motiven/get_perceptual_class

### References

- Bellas, F., Duro, R. J., Faiña, A., & Souto, D. (2010). Multilevel Darwinist Brain (MDB): Artificial evolution in a cognitive architecture for real robots. IEEE Transactions on autonomous mental development, 2(4), 340-354.
- Duro, R. J., Becerra, J. A., Monroy, J., & Calvo, L. (2017, June). Multilevel Darwinist Brain: Context Nodes in a Network Memory Inspired Long Term Memory. In International Work-Conference on the Interplay Between Natural and Artificial Computation (pp. 22-31). Springer, Cham.

## mdb_baxter
Tools for Baxter experiments

### Instalation:

- Create a workspace.
- Install baxter sdk dependencies: `apt install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers`
- Copy all the baxter related packages into the ws src folder and do `catkin_make`.
- Copy all the moveit related packages into the ws src folder and do `catkin_make`.
- Create simbolic links to the packages in the ws and do `catkin_make`.
- In the ws, do `catkin_make install`.

To check that the installation is good:
- Download this: `wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh`
- Permissions: `chmod u+x baxter.sh`
- Edit the machine ip or the hostname in the file (one of another, not both).
- Open a terminal and execute the script baxter.sh: `./baxter.sh sim`. This is needed in every terminal before launching any part of the code.
- In the same terminal after the script is launched: `roslaunch baxter_gazebo baxter_world.launch`. This will launch the simulated baxter.

When you see this in the terminal:
```
[ INFO] [1501860210.697581892, 34.007000000]: Simulator is loaded and started successfully
[ INFO] [1501860210.706354180, 34.015000000]: Robot is disabled
[ INFO] [1501860210.706493901, 34.015000000]: Gravity compensation was turned off

You can try on another terminal that has the baxter script already launched: `rosrun baxter_tools tuck_arms.py -u`. The robot will adquire the operation pose.

Now, to test moveit:
- On another enabled terminal: `rosrun baxter_interface joint_trajectory_action_server.py`
- On another enabled terminal: `roslaunch baxter_moveit_config baxter_grippers.launch`. Rviz should open with the planner.

### Launch:

To launch the tools for the real untucked robot run:

1. Xtion camera: `roslaunch openni2_launch openni2.launch rgb_frame_id:=camera_rgb_optical_frame depth_frame_id:=camera_depth_optical_frame depth_registration:=true auto_exposure:=false auto_white_balance:=false`
1. Dinamic tf between the camera and the robot: `roslaunch gii_baxter_detection camera_tf_link.launch`
1. Robot sensors: `roslaunch gii_baxter_detection robot_sense.launch decimation:=1`
1. Trajectory server: `rosrun baxter_interface joint_trajectory_action_server.py`
1. Moveit: `roslaunch baxter_moveit_config baxter_grippers.launch`
1. Getter auxiliary nodes: `roslaunch gii_baxter_moveit getters.launch`
1. Policies: `roslaunch gii_baxter_moveit baxter_policies.launch mode:=real`
1. Experiment manager: `roslaunch gii_baxter_exp exp_ltm_paris17.launch mode:=real`

To launch the tools for the simulated robot run:

1. Simulated baxter: `roslaunch baxter_gazebo baxter_world.launch`
1. Operation pose: `rosrun baxter_tools tuck_arms.py -u`
1. Trajectory server: `rosrun baxter_interface joint_trajectory_action_server.py`
1. Moveit: `roslaunch baxter_moveit_config baxter_grippers.launch`
1. Getter auxiliary nodes: `roslaunch gii_baxter_moveit getters.launch`
1. Policies: `roslaunch gii_baxter_moveit baxter_policies.launch`
1. Virtual sensors: `roslaunch gii_baxter_detection simulator_sense.launch`
1. Simulation manager: `roslaunch gii_baxter_exp exp_sim_manager.launch`
1. Experiment manager: `roslaunch gii_baxter_exp exp_ltm_paris17.launch`
