# ERC 2022 ArUco Checker

## Service

Checking service is advertised under `/erc_aruco_score`. It is defined in the `erc_aruco_checker` package (available [here](https://github.com/filesmuggler/erc_aruco_checker)). The request message type is of `ErcAruco`.
It is defined in the `erc_aruco_msg` package as 14 float32 arrays of 3 numbers, which correspond to 14 tag positions (tags from 1 to 14). On the scene. The returned type is a single float32 value with your obtained score. The tag placed inside the box on the panel is not taken into account.

## Frames
The ground truth is defined as transformation from `/base_link` frame of the robot to the tag name.

## Test
If you want to test your solution against the checker, you can clone the [repository](https://github.com/filesmuggler/erc_aruco_checker) which consists of the checker node, config files and launch file with parameters. 

## Training in simulation

Launch file `erc_aruco_checker.launch` starts `erc_aruco_checker.py` node. It takes two parameters:
- sim - boolean value if the checker is run against the simulated environment or on the real robot,
- tolerance - double value, which specifies tolerance of user detected position with respect to the ground truth.

You should call the service from within your code. The example of how to call a custom service with Python is pictured in the node `example_call.py` in the `erc_aruco_checker` package. The difference between real and simulated environments lies in the ground truth of the aruco tags placed in the simulation versus the tags in real life. 

