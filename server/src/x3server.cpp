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
using std::cout;
using std::cerr;
using std::endl;

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
        cout << "accept handle error=" << error.message() << endl;
}

void handle_wait(const boost::system::error_code& error,  
                     boost::asio::io_service& io) {
    if (!error)
        io.stop();
    else
        cout << "handle_wait error="<< error.message() << endl;
}

int main() {
    cout << "x3server starts at " << make_daytime_string() << endl;

    int cam_func_ret;
    size_t cam_nframe = 0;
    bool cam_is_on;
    accepted = false;
    cout << "X3 initializing..." << endl;
    cam_func_ret = manifold_cam_init(GETBUFFER_MODE | TRANSFER_MODE);
    if (cam_func_ret == -1) {
        cout << "X3 init error" << endl;
        return -1;
    }
    else {
        cout << "X3 inited" << endl;
    }
    
    cout << "X3 reading first frame..." << endl;
    cam_func_ret = manifold_cam_read(imgbuf, &cam_nframe, CAM_BLOCK);
    if (cam_func_ret == -1) {
        cout << "X3 first frame error" << endl;
        cam_is_on = false;
    }
    else {
        cout << "X3 first frame read" << endl;
        cam_is_on = true;
    }

    try {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(boost::asio::ip::address_v4::loopback(), PORT_NUMBER));
        while (cam_is_on) {
            tcp::socket socket(io_service);
            
            cout << "start accept..." << endl;
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
                cout << "accepted from " << socket.remote_endpoint() << " @ " << make_daytime_string() << endl;
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
                        cout << boost::format("#%d frame sent %d bytes") % frame_cnt % cnt << std::endl;
                    }
                    
                    if (!cnt) {
                        cout << "remote closed" << endl;
                        socket.close();
                        break;
                    }
                } catch (std::exception& e) {
                    cerr << "ERROR in while(1):" << endl;
                    cerr << e.what() << endl;
                    break;
                }
            }
            cout << "exit write loop..." << endl;
        }
        cout << "exit accept loop..." << endl;
    } catch (std::exception& e) {
        cerr << "ERROR outside while(1):" << endl;
        cerr << e.what() << endl;
    }

    cout << "exit x3..." << std::endl;
    while (!manifold_cam_exit()) {
        cout << "X3 failed to exit, retrying..." << endl;
        sleep(1);
    }
    cout << "X3 exit" << endl;

    return 0;
}
