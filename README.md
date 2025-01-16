# naosoccer_pos_action

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_humble.yaml?query=branch:iron)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

## Info

This repo contains some different files which execute .pos files for a NAO robot, a filetype defined in rUNSWift's codebase. To do this there is an action server and a node which shows an example of how a control node would contain a client, wrapping the client functionality with a subscriber and 2 publishers.
- `naosoccer_pos_action_node.hpp`, `naosoccer_pos_action_node.cpp`, and `naosoccer_pos_action_client.cpp` are all part of the example node that wraps the client. The client.cpp file is separate to clearly indicate an example client for easy copying, and the hpp has a clear section for the client as well.
- `naosoccer_pos_action_server.hpp` and `naosoccer_pos_action_server` contain the functionality of the action server which plays the pos files.

## Steps (on real robot)

1. On a terminal on the robot, run `ros2 run nao_lola_client nao_lola_client`
2. In a new terminal (either on robot, or on your machine), run `ros2 run naosoccer_pos_action naosoccer_pos_action_server`
3. In a new terminal (either on robot, or on your machine), run `ros2 run naosoccer_pos_action naosoccer_pos_action_node`
4. In a new terminal (either on robot, or on your machine), publish an action (name of a pos file less the .pos)

    `ros2 topic pub --once action_req std_msgs/msg/String '{data: "sit-to-stand"}'`

## Steps (simulation using rcsoccer3d)

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 run rcss3d_nao rcss3d_nao`
3. In a new terminal, run `ros2 run naosoccer_pos_action naosoccer_pos_action_server`
3. In a new terminal, run `ros2 run naosoccer_pos_action naosoccer_pos_action_node`
4. In a new terminal, publish an action (name of a pos file less the .pos)

    `ros2 topic pub --once action_req std_msgs/msg/String '{data: "sit-to-stand"}'`

## Using a different motion.

Pos files define the different motions, and can be specified using a different action_req. Simply change "sit-to-stand" above with a different pos file in this repo.

## Using custom motions.
To use custom motions, you can set a parameter to define the filepath to a custom pos folder. 
To set the parameter, when running the naosoccer_pos_action_server in the steps above, instead do either of the following:

```
ros2 run naosoccer_pos_action naosoccer_pos_action_server --ros-args -p "pos_folder:=/absolute-path-to-your-folder/pos/"
```
```
ros2 run naosoccer_pos_action naosoccer_pos_action_server --ros-args -p "pos_folder:=relative-path-to-your-folder-from-naosoccer_pos_action/pos/"
```

By default, upon the server initialisation it will attempt to parse all pos files recursively in the given folder to ensure they are the correct format. You can disable this with the parameter parse_on_initalise by running

```
ros2 run naosoccer_pos_action naosoccer_pos_action_server --ros-args -p "parse_on_initialise:=false"
```
