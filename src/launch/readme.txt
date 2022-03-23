In order to get the livox laser scanner running
your machine has to have a static IP address.
On possible option is:
    Adress: 192.168.1.2 Netmask: 255.255.255.0  Gateway: 192.168.1.1

In order for the encoder to work the phidget drivers
need to be compile from source with a minor adjustment in
the file
    phidgets_drivers/phidgets_high_speed_encoder/src/high_speed_encoder_ros_i.cpp
    line 198 -> 201: comment the for loop that calls setEnable for all channels
The drivers can be cloned from:
    https://github.com/ros-drivers/phidgets_drivers/tree/noetic
