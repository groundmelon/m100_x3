//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

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

int main(int argc, char* argv[]) {
    try {
        boost::asio::io_service io_service;

        tcp::endpoint endpoint(boost::asio::ip::address_v4::loopback(), PORT_NUMBER);

        // udp::socket socket(io_service);
        // socket.open(udp::v4());
        tcp::socket socket(io_service);
        boost::system::error_code error = boost::asio::error::host_not_found;
        std::cout << "Wait for acceptance... \n(Blocked here means server busy. Another x3client is running?)" << std::endl;
        socket.connect(endpoint, error);
        if (error) {
            std::cout << "Server offline, exit..." << std::endl;
            return EXIT_FAILURE;
        }

        boost::array<char, TOTAL_LEN> img;
        size_t imgidx = 0;
        size_t tfcnt = 0;
        ImageMsgHeader h;
        for (;;) {
            boost::array<char, TOTAL_LEN> buf;
            boost::system::error_code error;

            size_t len = socket.read_some(boost::asio::buffer(buf), error);

            if (error == boost::asio::error::eof) {
                std::cerr << "Server down" << std::endl;
                return EX_PROTOCOL;  // Connection closed cleanly by peer.
            } else if (error)
                throw boost::system::system_error(error);  // Some other error.
            // std::cout.write(buf.data(), len);
            // std::cout << "receive " << len << "bytes \t";

            std::copy(buf.begin(), buf.begin() + len, img.begin() + imgidx);
            imgidx += len;
            tfcnt++;
            if (imgidx == TOTAL_LEN) {
                h.parse((uint8_t*)img.c_array());
                struct timeval tm;
                gettimeofday(&tm, NULL);
                std::cout << bfmt("[%X] %dx%dx%d @ %.6f / %4d") % h.header %
                                 h.image_w % h.image_h % h.image_ch %
                                 ((double)h.secs + 1e-6 * (double)h.usecs) %
                                 // ((int)(tm.tv_sec-h.secs)*1000000+(int)(tm.tv_usec-h.usecs))
                                 tfcnt
                          << std::endl;
                imgidx = 0;
                tfcnt = 0;
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