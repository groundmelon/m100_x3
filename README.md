# M100 X3 ROS Driver

A ROS package which can read video stream from Zenmuse-X3-Camera on M100 and publish it as [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html). Must be compiled and launched on Manifold.

This package is based on [dji_sdk_manifold_read_cam](https://github.com/dji-sdk/Onboard-SDK-ROS/tree/2.3/dji_sdk_manifold_read_cam) and [dji-sdk/Manifold-Cam](https://github.com/dji-sdk/Manifold-Cam). Refer to them for more infomation.

### Steps for streaming video from X3 to Manifold, through SSH, without a hdmi-screen

1. Install all the prerequisites in [dji-sdk/Manifold-Cam](https://github.com/dji-sdk/Manifold-Cam)

2. (Optinal) Disable lightdm for stability:

    in `/etc/init/lightdm.conf`, line 12:
    
    Modify ```runlevel [!06]``` to ```runlevel [!026]```

3. Add ```xinit&``` to a startup script, such as:

    ```
    echo -e '#!/bin/bash\nxinit&' > /home/ubuntu/pre_x3
    chmod a+x /home/ubuntu/pre_x3
    ```
And add `/home/ubuntu/pre_x3` into `/etc/rc.local`

4. Add ```export DISPLAY=:0``` to your bashrc or remember to set this environment variable to THE USER WHO WILL RUN THE CODE

5. Run ```sudo -s``` to grant root privilege, because the driver needs to operate many `/dev` `/sys` stuffs

6. ```rosrun m100_x3 m100_x3_node```


### Steps for streaming video from X3 to Manifold with a hdmi-screen

Same as 1, 2, 6 above

