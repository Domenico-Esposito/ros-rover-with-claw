# 🤖 Rover 2WD with Claw

https://user-images.githubusercontent.com/3593408/162735472-cad62f20-62ee-4a8a-8f67-f33e0a3ab2fd.mp4


This page shows the main aspects of the robot. The robot uses Robot Operating System (ROS) runned on the RaspberryPi 2 Model B. The main goal of the robot is to grab, with a claw, a small object positioned in front of a wall with green background and release it in front another wall with purple background. The project is created for fun 🤓.

## Goal and Environment
The task of this robot is very simple: first of all go to the first target, next grab the object positioned in front of it, then find the second target, and finally release the object in front of this. The environment with run the task is composed of three element: a green wall, with a size 10.5 × 30 cm, a purple wall, with a size of 21 × 30 cm, and a small cylinder object, with a size of 6 × 8 cm.

## Hardware
- Raspberry Pi 2 Model B
- Rapberry Pi Camera v1.3
- TP-Link Wireless T2U Nano
- Motors servo SG90 (x2)
- DC Motors (x2)
- Bridge-H L9110S
- HC-SR04
- 330 Ohm resistor (x2)
- Rover base kit

## 3D Printed objects
- Claw and Servo box
- Ultrasound sensor support
- Chassis second floor and supports
- [Raspberry Pi Camera support](https://www.thingiverse.com/thing:2746186)

![Printed objects](./docs/img/printed.jpg "Printed objects")

## Connections
I have used the tool called [Fritzing](https://fritzing.org) for design the rover’s connections. In the design is not present the Pi Camera, because Fritzing doesn’t support this type of connection.

![Printed objects](./docs/img/fritzing.jpg "Printed objects")

## Software (ROS and Topics)
The core of the robot is a RaspberryPi 2 Model B with Ubuntu 18.16 LTS operation system. The version of ROS chosen is Noetic Ninjemys (ROS 1 LTS). This version is the most recent of ROS with native support to 32-bit ARM architecture.

I use five topics, showed in the following graph, for manage all aspects of rover. The sixth topic, "/camera_frame_with_detection" is only for debugging use, it shows the frame edit published from "/main_camera".

![ROS Graph of rover](./docs/img/rosgraph.jpg "ROS Graph")

## Flowchart of Rover
![Flowchart of rover](./docs/img/software_flowchart.jpg "Flowchart")

## 📷 Images
![Rover](./docs/img/rover/rover_7.jpg "Rover 7")

![Rover](./docs/img/rover/rover_1.jpg "Rover 1")

![Rover](./docs/img/rover/rover_2.jpg "Rover 2")

![Rover](./docs/img/rover/rover_3.jpg "Rover 3")

![Rover](./docs/img/rover/rover_4.jpg "Rover 4")

![Rover](./docs/img/rover/rover_5.jpg "Rover 5")

![Rover](./docs/img/rover/rover_6.jpg "Rover 6")

![Rover](./docs/img/rover/rover_8.jpg "Rover 8")

