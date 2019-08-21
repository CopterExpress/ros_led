# LED strip support for ROS

## led_msgs

Common messages for LEDs.

## ros_ws281x package

### Running without root permissions

To allow running the node without root permissions set the *[setuid](https://en.wikipedia.org/wiki/Setuid)* bit to the executable:

```bash
sudo chown root:root `catkin_find ros_ws281x ros_ws281x_node`
sudo chmod +s `catkin_find ros_ws281x ros_ws281x_node`
```
