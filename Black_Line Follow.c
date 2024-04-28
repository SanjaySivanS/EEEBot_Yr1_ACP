#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "opencv_aee.hpp"
#include "main.hpp"
#include "pi2c.h"
#include <iostream>
#include <math.h>

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void) {
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

void rgb2hsv(int r, int g, int b, int *h, int *s, int *v) {
    float minval, maxval, rf, bf, gf, diff;
    rf = r / 255.0f;
    bf = b / 255.0f;
    gf = g / 255.0f;
    maxval = fmaxf(rf, fmaxf(gf, bf));
    minval = fminf(rf, fminf(gf, bf));
    diff = maxval - minval;

    if (maxval == minval) {
        *h = 0;
    } else if (maxval == gf) {
        *h = (int)(60 * (((bf - rf) / diff) + 2));
    } else if (maxval == rf) {
        *h = (int)(60 * fmodf((gf - bf) / diff, 6));
    } else if (maxval == bf) {
        *h = (int)(60 * (((rf - gf) / diff) + 4));
    }

    if (*h < 0) {
        *h += 360;
    }

    if (maxval == 0) {
        *s = 0;
    } else {
        *s = (int)((diff / maxval) * 100);
    }

    *v = (int)(maxval * 100);
}

int main(int argc, char** argv) {
    setup();    // Call a setup function to prepare IO and devices
    int file;
    const char *filename = "/dev/i2c-1";
    int address = 0x08;
    file = open(filename, O_RDWR);
    ioctl(file, I2C_SLAVE, address);

    cv::namedWindow("Colour view: Black");
    cv::namedWindow("Sensor View");

    int r, g, b;
    int x = 0, y = 0;
    int h, s, v;
    float Xpk = 0;
    float denominator = 0, numerator = 0;
    int whitePixels[196] = {0};

    while(1) {
        cv::Mat frame;
        while (frame.empty())
            frame = captureFrame();

        cv::Mat colourDisp(40, 196, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat sensorView(40, 196, CV_8UC3, cv::Scalar(0, 0, 0));
        
        for (x = 0; x < 196; x++) {
            for (y = 0; y < 40; y++) {
                r = frame.at<cv::Vec3b>(y, x)[2];
                g = frame.at<cv::Vec3b>(y, x)[1];
                b = frame.at<cv::Vec3b>(y, x)[0];
                
                rgb2hsv(r, g, b, &h, &s, &v);

                if (v <= 35 && s <= 15 || v <= 20) {
                    colourDisp.at<cv::Vec3b>(Point(x, y)) = {255, 255, 255};
                    whitePixels[x] += 1;
                } else {
                    colourDisp.at<cv::Vec3b>(Point(x, y)) = {0, 0, 0};
                }

                sensorView.at<cv::Vec3b>(Point(x, y)) = {b, g, r};
            }
        }

        Xpk = 0.0f;
        denominator = 0.0f;
        numerator = 0.0f;

        for (int i = 0; i < 196; i++) {
            denominator += whitePixels[i] * (i - 98);
            numerator += whitePixels[i];
        }

        Xpk = denominator / numerator;

        printf("Xpk = %f\n", Xpk);

        unsigned char buf[sizeof(float)];
        memcpy(buf, &Xpk, sizeof(float));
        write(file, buf, sizeof(float));

        cv::imshow("Photo", frame);
        cv::imshow("Colour view: Black", colourDisp);
        cv::imshow("Sensor View", sensorView);

        int key = cv::waitKey(1);
        key = (key == 255) ? -1 : key;

        if (key == 27) {
            break;
        }
    }

    close(file);
    closeCV();  // Disable the camera and close any windows
    return 0;
}
