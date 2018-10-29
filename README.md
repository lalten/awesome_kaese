# Awesome KaeSE

[Hackathon Stuttgart](https://www.hackathon-stuttgart.de) 2018 winner of "Best Implementation" and "Kärcher Challenge" awards

Remotely control a Kaercher KM 85/50 R Kehrmaschine

Presentation: [Video](https://www.facebook.com/HackathonStg/videos/478566952632191/?t=1480), [Slides](https://docs.google.com/presentation/d/1kymEtSE9jgL6B5Tpqj0swS6tOIg3Bon7CwWrfvAZgis/edit?usp=sharing)

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
