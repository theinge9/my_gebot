## GeBot's Description : Tricycle Steering Robot 

Model:URDF file for GeBot for Simulation Test
Drive and Steering Control to just front_wheel_joint

Simulation Engine : Gazebo Classic / xIgnition:Fortress

Launch and Test : 
```
ros2 launch my_gebot sim.launch.py
```
Controller Load : 
```
ros2 run controller_manager spawner tricycle_controller               
ros2 run controller_manager spawner joint_state_broadcaster
```                
Teleop : 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tricycle_controller/cmd_vel
```
Slam :
```
ros2 launch my_gebot slam.launch.py
```
Velodyne : Need to install velodyne_simulator.git
