#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "cv.h"

#include "djicam.h"
#include "djicam_utils.h"

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE (IMAGE_W * IMAGE_H * 3)

#define mode (GETBUFFER_MODE | TRANSFER_MODE)

void fill_camera_info_message_with_default_value(
    sensor_msgs::CameraInfo& cam_info) {
    cam_info.header.frame_id = "/camera";
    cam_info.distortion_model = "";
    cam_info.D = {-0.1297646493949856, 0.0946885697670611,
                  -0.0002935002712265514, -0.00022663675362156343, 0.0};
    cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0,
                  518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0,
                  504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0,
                  0.0};
    cam_info.binning_x = 0;
    cam_info.binning_x = 0;

    cam_info.roi.x_offset = 0;
    cam_info.roi.y_offset = 0;
    cam_info.roi.height = 0;
    cam_info.roi.width = 0;
    cam_info.roi.do_rectify = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "m100_x3");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport transport(nh);
    image_transport::Publisher image_pub;
    ros::Publisher caminfo_pub;
    sensor_msgs::Image im;
    sensor_msgs::CameraInfo cam_info;
    cv_bridge::CvImage cvi;

    IplImage* pRawImg;
    IplImage* pImg;
    unsigned char* pData;

    int dest_image_w, dest_image_h;
    int ret;
    unsigned char buffer[FRAME_SIZE] = {0};
    unsigned int nframe = 0;

    bool enable_color = false;
    bool enable_camera_info = false;
    int downsample_factor = 0;

    nh.param("enable_color", enable_color, false);
    nh.param("downsample_factor", downsample_factor, 2);
    nh.param("camera_info", enable_camera_info, false);

    ros::Time now_time = ros::Time::now();
    cvi.header.frame_id = "image";
    cvi.encoding = enable_color ? "bgr8" : "mono8";

    if (downsample_factor > 1) {
        dest_image_w = IMAGE_W / downsample_factor;
        dest_image_h = IMAGE_H / downsample_factor;
    } else {
        dest_image_w = IMAGE_W;
        dest_image_h = IMAGE_H;
    }

    if (enable_color) {
        pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 3);
        pImg =
            cvCreateImage(cvSize(dest_image_w, dest_image_h), IPL_DEPTH_8U, 3);
        pData = new unsigned char[IMAGE_W * IMAGE_H * 3];
    } else {
        pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 1);
        pImg =
            cvCreateImage(cvSize(dest_image_w, dest_image_h), IPL_DEPTH_8U, 1);
    }

    image_pub = transport.advertise("image_raw", 1);
    if (enable_camera_info) {
        caminfo_pub =
            nh.advertise<sensor_msgs::CameraInfo>("dji_sdk/camera_info", 1);
        cam_info.width = dest_image_w;
        cam_info.height = dest_image_h;
        fill_camera_info_message_with_default_value(cam_info);
    };

    ROS_INFO("[M100-X3] ImageType: %s | ImageSize: %dx%d | CameraInfo: %s",
             enable_color ? "BGR" : "Grayscale", dest_image_w, dest_image_h,
             enable_camera_info ? "ON" : "OFF");

    ROS_INFO("[M100-X3] Start X3 initialization.");
    ret = manifold_cam_init(mode);
    if (ret == -1) {
        ROS_ERROR("[M100-X3] X3 init error. \n");
        return -1;
    } else {
        ROS_INFO("[M100-X3] X3 init success!\n");
    }

    ros::Rate r(200.0);
    while (ros::ok()) {
        r.sleep();
        now_time = ros::Time::now();
        ret = manifold_cam_read(buffer, &nframe, CAM_NON_BLOCK);
        ROS_DEBUG("[M100-X3] manifold_cam_read(): ret=%d", ret);
        if (ret == 0) {
            // pass
        } else if (ret == -1) {
            break;
        } else if (ret == (FRAME_SIZE / 2)) {
            // (FRAME_SIZE/2) == 1382400, which is the expect ret value
            if (enable_color) {
                NV12ToRGB(buffer, pData, 1280, 720);
                memcpy(pRawImg->imageData, pData, FRAME_SIZE);

            } else {
                memcpy(pRawImg->imageData, buffer, FRAME_SIZE / 3);
            }

            if (IMAGE_W == dest_image_w && IMAGE_H == dest_image_h) {
                cvi.image = pRawImg;
            } else {
                cvResize(pRawImg, pImg, CV_INTER_LINEAR);
                cvi.image = pImg;
            }

            cvi.header.stamp = now_time;
            cvi.toImageMsg(im);
            image_pub.publish(im);

            if (enable_camera_info) {
                cam_info.header.stamp = now_time;
                caminfo_pub.publish(cam_info);
            }
        } else {
            ROS_WARN("[M100-X3] manifold_cam_read(): ret=%d", ret);
        }
    }

    while (!manifold_cam_exit()) {
        ROS_WARN("[M100-X3] X3 failed to exit, retrying...");
        sleep(1);
    }

    ROS_INFO("[M100-X3] Exit...");
    sleep(1);
    return 0;
}
