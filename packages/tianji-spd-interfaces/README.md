# tianji_spd_interfaces

Single-source ROS 2 interface for the SPD Tianji/Wuji simulation boundary.

`JointCommand` is an absolute 54-DoF position target in radians. The fixed wire
order is left arm 7, right arm 7, left hand 20, right hand 20. The publisher
must send the complete name and position arrays on every frame.
