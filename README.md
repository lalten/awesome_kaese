# Awesome KaeSE

Hackathon Stuttgart 2018

Idea: Control Kaercher KM 85/50 R Kehrmaschine

## Nodes

### kaercher_hwif
Subscribes to
* `/cmd_vel` - geometry_msgs/Twist. Requested velocity, steering angle
* `/override` - std_msgs/Bool. Immediately halt if True

Important params:
* `controller_p` - steering controller P gain

Prerequisites:
 * pip install tinkerforge
 * Install [brickd](https://www.tinkerforge.com/de/doc/Software/Brickd.html), [brickv](https://www.tinkerforge.com/de/doc/Software/Brickv.html)
 
### joy_cmd_vel
Reads Tinkerforge Joystick, publishes `/cmd_vel`
