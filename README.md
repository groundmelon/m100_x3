# M100 X3 ROS Driver

A ROS package which can read video stream from Zenmuse-X3-Camera on M100 and publish it as [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html). Must be compiled and launched on Manifold.

Two types for running this package: [manully launch ROS node](https://github.com/groundmelon/m100_x3/blob/master/README.md#steps-for-manully-launched-ros-driver) or [server/client type](https://github.com/groundmelon/m100_x3/blob/master/README.md#steps-for-manully-launched-ros-driver). First one is simple, and second one is more convenient.

This package is based on [dji_sdk_manifold_read_cam](https://github.com/dji-sdk/Onboard-SDK-ROS/tree/2.3/dji_sdk_manifold_read_cam) and [dji-sdk/Manifold-Cam](https://github.com/dji-sdk/Manifold-Cam). Read their documents and codes before continuing.

## Background and Technical Detail

Manifold will cut off video signal between gimbal camera and video transmitter, so you will NOT see the live video on the DJI GO App, until the camera driver is launched in transfer mode. But this is not convenient if you don't want to bring a laptop with the drone, and ssh to manifold, and launch the driver manually to view video in DJI GO App.

Obviously, it's better to run the driver as a background daemon at system startup than run it manually every time. However, the driver is not reentrant, which means that if a background driver instance is running, we can not run another driver instance to get the video (and show/publish to ros).

In order to solve this problem, a server which provides video acqusition functionality is provided in this package. And a manually launched driver with ros interface is also provided.

For manually launched driver, the source code is [src/m100_x3_node.cpp](https://github.com/groundmelon/m100_x3/blob/master/src/m100_x3_node.cpp). This file will initialize the X3 camera driver, read video data and publish the data as sensor_msgs/Image. Since the initialization need root privilege, you must excute ```sudo -s``` to grant root privilege before rosrun/roslaunch.

For server type driver, the server source code is [src/server/src/x3server.cpp](https://github.com/groundmelon/m100_x3/blob/master/server/src/x3server.cpp). The server will be launched automatically by the __upstart__ system in ubuntu after installation. The server will initialize the x3 camera driver, and open a TCP port at ```localhost:40042```, blocked at socket accept function. When a client is connected, the accept function will return, and the server will get video data from the initialized driver and send the data to the client. If the connection with client is terminated, the server will go back to accept mode for the next client. Currently, server only accpets on client at the same time.

The client source code is [src/m100_x3_client_node.cpp](https://github.com/groundmelon/m100_x3/blob/master/src/m100_x3_client_node.cpp). The client will try to connect to localhost:40042, and read data and interpret it as a predefined size-and-type image, and publish it as ros message.

Attention: after x3 gimbal camera driver is initialized, it will have a 50% usage of one cpu core. Please consider carefully if a 50%-cpu-usage-service will affect the performance of other programs


## Steps For Manully Launched ROS Driver
### If you want to streaming video from X3 to Manifold, through SSH, without a hdmi-screen

1. Install all the prerequisites in [dji-sdk/Manifold-Cam](https://github.com/dji-sdk/Manifold-Cam) (Mainly aims to install libdcam.so in the system)

2. (Optional or Necessary? I'm not sure) Disable lightdm for stability:

    in `/etc/init/lightdm.conf`, line 12:
    
    Modify ```runlevel [!06]``` to ```runlevel [!026]```

3. Add ```xinit&``` to a startup script, such as:

    ```
    echo -e '#!/bin/bash\nxinit&' > /home/ubuntu/pre_x3
    chmod a+x /home/ubuntu/pre_x3
    ```
   And add `/home/ubuntu/pre_x3` into `/etc/rc.local`

4. Add ```export DISPLAY=:0``` to your bashrc or remember to set this environment variable to THE USER WHO WILL RUN THE CODE

5. (Maybe a reboot here)

6. Run ```sudo -s``` to grant root privilege, because the driver needs to operate many `/dev` `/sys` stuffs

7. ```rosrun m100_x3 m100_x3_node```


### If you want to streaming video from X3 to Manifold with a hdmi-screen

Same as 1, 6, 7 above

## Steps For System Service and Client Type

1. Install all the prerequisites in [dji-sdk/Manifold-Cam](https://github.com/dji-sdk/Manifold-Cam) (Mainly aims to install libdcam.so in the system)

2. cd src/server && make && sudo make install. This step will install the essential scripts and executables into the system to utilize the startup system.

3. You can use ```sudo service x3server start/stop/status``` to operate the x3 driver service. Useful output/log can be seen from /var/log/upstart/x3server.log

4. To prevent automatic startup, edit /etc/init/x3server.conf, in start on item, change [!06] to [!0123456]

## Some bugs and todos

1. The dcam library uses some unusual techniques to handle SIGINT/SIGTERM to shutdown. And combined with socket, lots of efforts were devoted to handle termination of the server, but it is still not perfect. So there is a chance to fail on stopping the server. When the server is not running properly, a system reboot is recommanded.

2. Now, the server and client only support full-size grayscale image. Support of RGB and resized image should be added.

3. The port number 40042 is hardcoded in src/server/include/msgdef.h. Maybe it needs to change to other port number if this one is invalid, and the client should be able to find out the new port number.

## Backup for Manifold Resources

djicdn links:

[User Manual](https://dl.djicdn.com/downloads/manifold/en/Manifold_User_Manual_en_v1.0.pdf)
    
[用户手册](https://dl.djicdn.com/downloads/manifold/cn/Manifold_User_Manual_cn_v1.0.pdf)
    
[manifold_image_v1.0.tar.gz](https://dl.djicdn.com/downloads/manifold/manifold_image_v1.0.tar.gz)
    
[manifold_kernel_source_v1.0.tar.gz](https://dl.djicdn.com/downloads/manifold/en/manifold_kernel_source_v1.0.tar.gz)

google-drive:
    https://drive.google.com/open?id=0By08HNZRtOAYSFpPVWtFWnBfdmc
    
[How to add ft232 driver](http://elinux.org/Jetson/Tutorials/Program_An_Arduino)
