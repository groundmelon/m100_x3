#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "cv.h"

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <sysexits.h>
#include "msgdef.h"

using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using bfmt = boost::format;

using std::cout;
using std::endl;

int main(int argc, char** argv) {
    ros::init(argc, argv, "m100_x3_client");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport transport(nh);
    image_transport::Publisher image_pub;
    ros::Publisher caminfo_pub;
    sensor_msgs::Image im;
    sensor_msgs::CameraInfo cam_info;
    cv_bridge::CvImage cvi;

    cvi.header.frame_id = "image";
    cvi.encoding = "mono8";

    image_pub = transport.advertise("image_raw", 1);

    IplImage* pImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 1);

    try {
        boost::asio::io_service io_service;

        tcp::endpoint endpoint(boost::asio::ip::address_v4::loopback(), PORT_NUMBER);

        // udp::socket socket(io_service);
        // socket.open(udp::v4());
        tcp::socket socket(io_service);
        boost::system::error_code error = boost::asio::error::host_not_found;
        ROS_INFO_STREAM(
            "Wait for acceptance... \n(Blocked here means server busy. Another x3client is "
            "running?)");
        socket.connect(endpoint, error);
        if (error) {
            ROS_ERROR_STREAM("Server offline, exit...");
            return EXIT_FAILURE;
        }

        boost::array<char, TOTAL_LEN> img;
        size_t imgidx = 0;
        size_t tfcnt = 0;
        ImageMsgHeader h;
        ROS_INFO("Connect successfully.");
        while (ros::ok()) {
            boost::array<char, TOTAL_LEN> buf;
            boost::system::error_code error;

            size_t len = socket.read_some(boost::asio::buffer(buf), error);

            if (error == boost::asio::error::eof) {
                ROS_ERROR_STREAM("Server down");
                return EX_PROTOCOL;  // Connection closed cleanly by peer.
            } else if (error)
                throw boost::system::system_error(error);  // Some other error.

            std::copy(buf.begin(), buf.begin() + len, img.begin() + imgidx);
            imgidx += len;
            tfcnt++;
            if (imgidx == TOTAL_LEN) {
                h.parse((uint8_t*)img.c_array());
                // struct timeval tm;
                // gettimeofday(&tm, NULL);
                ROS_DEBUG_STREAM(bfmt("[%X] %dx%dx%d @ %.6f / %4d") % h.header % h.image_w %
                                 h.image_h % h.image_ch %
                                 ((double)h.secs + 1e-6 * (double)h.usecs) %
                                 // ((int)(tm.tv_sec-h.secs)*1000000+(int)(tm.tv_usec-h.usecs))
                                 tfcnt);
                imgidx = 0;
                tfcnt = 0;
                if ((h.header == 0XFEDCBA98) && (h.image_w == IMAGE_W) && (h.image_h == IMAGE_H) &&
                    (h.image_ch == IMAGE_CH)) {
                    memcpy(pImg->imageData, (uint8_t*)img.c_array() + HEADER_LEN, IMG_DATA_LEN);
                    cvi.image = pImg;
                    cvi.header.stamp = ros::Time(h.secs, h.usecs);
                    cvi.toImageMsg(im);
                    image_pub.publish(im);
                } else {
                    std::cerr << "Header not correct!!!!" << std::endl;
                    return EX_IOERR;
                }

            } else if (imgidx > TOTAL_LEN) {
                std::cerr << "receive overflow!!!!" << std::endl;
                return EX_IOERR;
            } else {
                // std::cout << std::endl;
            }
        }
    } catch (std::exception& e) {
        std::cout << "Big error" << std::endl;
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
