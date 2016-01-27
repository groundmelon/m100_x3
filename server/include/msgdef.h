#ifndef __MSGDEF_H
#define __MSGDEF_H 1

#include <boost/asio.hpp>

static const int PORT_NUMBER = 40042;

static const uint32_t IMAGE_W = 1280;
static const uint32_t IMAGE_H = 720;
static const uint32_t IMAGE_CH = 1;
static const uint32_t HEADER_LEN = 1 * sizeof(uint32_t) + 3 * sizeof(uint32_t) + 2 * sizeof(uint32_t);
static const uint32_t TOTAL_LEN = HEADER_LEN+IMAGE_W*IMAGE_H*IMAGE_CH;

class ImageMsgHeader {
public:
    uint32_t header;
    uint32_t image_w;
    uint32_t image_h;
    uint32_t image_ch;
    uint32_t secs;
    uint32_t usecs;
    void parse(uint8_t * p) {
        header   = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
        image_w  = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
        image_h  = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
        image_ch = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
        secs     = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
        usecs    = ntohl(*(uint32_t *)p); p += sizeof(uint32_t);
    }
    void pack(uint8_t * p) {
        *(uint32_t *)p = htonl(header  ); p += sizeof(uint32_t);
        *(uint32_t *)p = htonl(image_w ); p += sizeof(uint32_t);
        *(uint32_t *)p = htonl(image_h ); p += sizeof(uint32_t);
        *(uint32_t *)p = htonl(image_ch); p += sizeof(uint32_t);
        *(uint32_t *)p = htonl(secs    ); p += sizeof(uint32_t);
        *(uint32_t *)p = htonl(usecs   ); p += sizeof(uint32_t);   
    }
};

#endif
