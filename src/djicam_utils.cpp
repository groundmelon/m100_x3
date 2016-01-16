#include "djicam_utils.h"
#include <string.h>

sRGB yuvTorgb(int Y, int U, int V) {
    sRGB rgb;
    rgb.r = (int)(Y + 1.4075 * (V - 128));
    rgb.g = (int)(Y - 0.3455 * (U - 128) - 0.7169 * (V - 128));
    rgb.b = (int)(Y + 1.779 * (U - 128));
    rgb.r = (rgb.r < 0 ? 0 : rgb.r > 255 ? 255 : rgb.r);
    rgb.g = (rgb.g < 0 ? 0 : rgb.g > 255 ? 255 : rgb.g);
    rgb.b = (rgb.b < 0 ? 0 : rgb.b > 255 ? 255 : rgb.b);
    return rgb;
}

unsigned char* NV12ToRGB(unsigned char* src, unsigned char* rgb, int width,
                         int height) {
    int numOfPixel = width * height;
    int positionOfU = numOfPixel;
    int startY, step, startU, Y, U, V, index, nTmp;
    sRGB tmp;

    for (int i = 0; i < height; i++) {
        startY = i * width;
        step = i / 2 * width;
        startU = positionOfU + step;
        for (int j = 0; j < width; j++) {
            Y = startY + j;
            if (j % 2 == 0)
                nTmp = j;
            else
                nTmp = j - 1;
            U = startU + nTmp;
            V = U + 1;
            index = Y * 3;
            tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
            rgb[index + 0] = (char)tmp.b;
            rgb[index + 1] = (char)tmp.g;
            rgb[index + 2] = (char)tmp.r;
        }
    }
    return rgb;
}

bool YUV420_To_BGR24(unsigned char* puc_y, unsigned char* puc_u,
                     unsigned char* puc_v, unsigned char* puc_rgb, int width_y,
                     int height_y) {
    if (!puc_y || !puc_u || !puc_v || !puc_rgb) {
        return false;
    }
    int baseSize = width_y * height_y;
    int rgbSize = baseSize * 3;

    BYTE* rgbData = new BYTE[rgbSize];
    memset(rgbData, 0, rgbSize);

    int temp = 0;

    BYTE* rData = rgbData;
    BYTE* gData = rgbData + baseSize;
    BYTE* bData = gData + baseSize;

    int uvIndex = 0, yIndex = 0;

    for (int y = 0; y < height_y; y++) {
        for (int x = 0; x < width_y; x++) {
            uvIndex = (y >> 1) * (width_y >> 1) + (x >> 1);
            yIndex = y * width_y + x;

            temp = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
            rData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

            temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
                         (puc_v[uvIndex] - 128) * (-0.7145));
            gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

            temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
            bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
        }
    }

    int widthStep = width_y * 3;
    for (int y = 0; y < height_y; y++) {
        for (int x = 0; x < width_y; x++) {
            puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x];  // R
            puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x];  // G
            puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x];  // B
        }
    }

    if (!puc_rgb) {
        return false;
    }
    delete[] rgbData;
    return true;
}

// IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height)
// {
// 	if (!pYUV420)
// 	{
// 		return NULL;
// 	}

// 	int baseSize = width*height;
// 	int imgSize = baseSize*3;
// 	BYTE* pRGB24 = new BYTE[imgSize];
// 	memset(pRGB24, 0, imgSize);

// 	int temp = 0;

// 	BYTE* yData = pYUV420;
// 	BYTE* uData = pYUV420 + baseSize;
// 	BYTE* vData = uData + (baseSize>>2);

// 	if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false
// || !pRGB24)
// 	{
// 		return NULL;
// 	}

// 	IplImage *image = cvCreateImage(cvSize(width, height), 8,3);
// 	memcpy(image->imageData, pRGB24, imgSize);

// 	if (!image)
// 	{
// 		return NULL;
// 	}

// 	delete [] pRGB24;
// 	return image;
// }
