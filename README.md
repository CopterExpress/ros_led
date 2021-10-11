## LED STRIP WS2812 WITH RASPI 4(tested on ROS Noetic)
This Fork is for GPIOS 13, 19, 41, 45 or 53(channel 1)
The default repo works best for channel 0

Install the library from source:

```
https://github.com/jgarff/rpi_ws281x
```

NOTE: Make sure you do all the configuration in this section(https://github.com/jgarff/rpi_ws281x#limitations) for PWM/PCM/SPI type GPIO(only tested on pin 13)

```
cd /rpi_ws281x/examples/

gedit strandtest.py
```

``NOTE: Set correct parameters(LED_PIN, LED_COUNT, LED_CHANNEL) in the file.``
```
sudo python3 strandtest.py (make sure this is working)

cd ~/catkin_ws/src
clone https://github.com/CopterExpress/ros_led.git
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release

source ~/.bashrc (hope your bashrc has correct path to setup files)

#to allow the node to run without root permission
sudo chown root:root $(catkin_find ws281x ws281x_node)
sudo chmod +s $(catkin_find ws281x ws281x_node)

roslaunch ws281x example.launch
```

# IN CASE OF THESE ERRORS:

error while loading shared libraries: librosconsole_log4cxx.so: cannot open shared object file: No such file or directory
error while loading shared libraries: libxmlrpcpp.so: cannot open shared object file: No such file or directory

```
sudo apt install libxmlrpcpp-dev
sudo vi /etc/ld.so.conf --> Add /usr/local/lib and /opt/ros/noetic/lib in the next line
sudo /sbin/ldconfig -v
sudo ldconfig
sudo reboot
```
# IN CASE YOU GET THIS:
```
Gpio 13 is illegal for LED channel 0
[ws281x] native library init failed: Selected GPIO not possible
```
Make sure you have set correct channel number in the source code.
It is Channel 1 for GPIOs 13, 19, 41, 45 or 53 and rest of the GPIOs must be channel 0 (Best bet is use the default package for channel 0)


