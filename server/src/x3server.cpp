#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <sysexits.h>

#include "msgdef.h"
#include "djicam.h"

const size_t FRAME_SIZE = 1280*720*3;

using boost::asio::ip::udp;
using boost::asio::ip::tcp;

typedef boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_SNDBUF> send_buf_opt_t;

uint8_t imgbuf[FRAME_SIZE];
uint8_t sockbuf[TOTAL_LEN];
bool accepted;

std::string make_daytime_string() {
    using namespace std;  // For time_t, time and ctime;
    time_t now = time(0);
    return ctime(&now);
}

std::ofstream * pOS;

void accept_handle(const boost::system::error_code& error) {
    if (!error) 
        accepted = true;
    else
        (*pOS) << "accept handle error=" << error.message() << std::endl;
}

void handle_wait(const boost::system::error_code& error,  
                     boost::asio::io_service& io) {
    if (!error)
        io.stop();
    else
        std::cout << "handle_wait error="<< error.message() << std::endl;
}

int main() {
    std::ofstream logstream("/tmp/x3server.log");
    if (!logstream.is_open()) {
        return EX_CANTCREAT;
    }

    pOS = &logstream;
    logstream << "x3server starts at " << make_daytime_string() << std::endl;

    int cam_func_ret;
    size_t cam_nframe = 0;
    bool cam_is_on;
    accepted = false;
    logstream << "X3 initializing..." << std::endl;
    cam_func_ret = manifold_cam_init(GETBUFFER_MODE | TRANSFER_MODE);
    if (cam_func_ret == -1) {
        logstream << "X3 init error" << std::endl;
        return -1;
    }
    else {
        logstream << "X3 inited" << std::endl;
    }
    
    logstream << "X3 reading first frame..." << std::endl;
    cam_func_ret = manifold_cam_read(imgbuf, &cam_nframe, CAM_BLOCK);
    if (cam_func_ret == -1) {
        logstream << "X3 first frame error" << std::endl;
        return -1;
    }
    else {
        logstream << "X3 first frame read" << std::endl;
        cam_is_on = true;
    }

    try {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(boost::asio::ip::address_v4::loopback(), PORT_NUMBER));
        while (cam_is_on) {
            tcp::socket socket(io_service);
            
            logstream << "start accept..." << std::endl;
            accepted = false;
            while(!accepted && cam_is_on) {
                io_service.reset();
                acceptor.async_accept(socket, accept_handle);
                boost::asio::deadline_timer timer(io_service, boost::posix_time::seconds(3));
                timer.async_wait(boost::bind(handle_wait,   
                                 boost::asio::placeholders::error,  
                                 boost::ref(io_service)));
                io_service.run();
                // usleep(1000000/2);
                cam_func_ret = manifold_cam_read(imgbuf, &cam_nframe, CAM_NON_BLOCK);
                if (cam_func_ret==-1) {
                    cam_is_on = false;
                    break;
                }
            }
            
            if (!cam_is_on) {
                break;
            }

            if (accepted) {
                logstream << "accepted from " << socket.remote_endpoint() << " @ " << make_daytime_string() << std::endl;
            }
            

            // send_buf_opt_t opt1(212992);
            // socket.set_option(opt1);

            int frame_cnt = 0;
            struct timeval tm;
            ImageMsgHeader h;
            h.header = 0xFEDCBA98;
            h.image_w = IMAGE_W;
            h.image_h = IMAGE_H;
            h.image_ch = IMAGE_CH;
            while (cam_is_on) {
                cam_func_ret = manifold_cam_read(imgbuf, &cam_nframe, CAM_NON_BLOCK);
                if (cam_func_ret==-1) {
                    cam_is_on = false;
                    break;
                }
                else if (cam_func_ret == 0) {
                    usleep(1000);
                    continue;
                }
                memcpy(sockbuf+HEADER_LEN, imgbuf, FRAME_SIZE / 3);
                try {
                    frame_cnt++;
                    boost::asio::mutable_buffers_1 message = boost::asio::buffer(sockbuf, TOTAL_LEN);

                    gettimeofday(&tm, NULL);
                    h.secs = tm.tv_sec;
                    h.usecs = tm.tv_usec;
                    
                    h.pack((uint8_t *)sockbuf);
                    // std::string message = make_daytime_string();

                    boost::system::error_code ignored_error;
                    int cnt = boost::asio::write(socket, boost::asio::buffer(message),
                                       boost::asio::transfer_all(), ignored_error);

                    // logout every frame for first 1 sec, every sec for first 1 minutes, and every minute 
                    if (frame_cnt<30 || (frame_cnt<1800 && frame_cnt%30==0) || frame_cnt%1800==0) {
                        logstream << boost::format("#%d frame sent %d bytes") % frame_cnt % cnt << std::endl;
                    }
                    
                    if (!cnt) {
                        logstream << "remote closed" << std::endl;
                        socket.close();
                        break;
                    }
                } catch (std::exception& e) {
                    logstream << "ERROR in while(1):" << std::endl;
                    logstream << e.what() << std::endl;
                    break;
                }
            }
            logstream << "exit write loop..." << std::endl;
        }
        logstream << "exit accept loop..." << std::endl;
    } catch (std::exception& e) {
        logstream << "ERROR outside while(1):" << std::endl;
        logstream << e.what() << std::endl;
    }

    logstream << "exit x3..." << std::endl;
    while (!manifold_cam_exit()) {
        logstream << "X3 failed to exit, retrying..." << std::endl;
        sleep(1);
    }
    logstream << "X3 exit" << std::endl;

    return 0;
}
