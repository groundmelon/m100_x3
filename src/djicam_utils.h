#ifndef __DJICAM_UTILS_H
#define __DJICAM_UTILS_H

typedef unsigned char BYTE;

struct sRGB{
	int r;
	int g;
	int b;
};

sRGB yuvTorgb(int Y, int U, int V);
unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height);
bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y);
// IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height);

#endif