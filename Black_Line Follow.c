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

void setup() {
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

bool isNumber(float x) {
    return (x == x); // NaN (not a number) is not equal to itself
}

void rgb2hsv(int r, int g, int b, int* h, int* s, int* v) {
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

    cv::Mat umbrella = cv::imread("Umbrella.png", cv::IMREAD_GRAYSCALE);
    cv::Mat triangle = cv::imread("Triangle.png", cv::IMREAD_GRAYSCALE);
    cv::Mat circle = cv::imread("Circle.png", cv::IMREAD_GRAYSCALE);
    cv::Mat star = cv::imread("Star.png", cv::IMREAD_GRAYSCALE);

    int file;
    const char* filename = "/dev/i2c-1";
    int address = 0x08;

    file = open(filename, O_RDWR);
    ioctl(file, I2C_SLAVE, address);

    cv::namedWindow("Photo");
    cv::namedWindow("Colour view: Pink");

    float Xpk = 0.0f;
    float denominator = 0.0f;
    float numerator = 0.0f;

    bool followblack = true;
    char symbol;

    int blackIntensity[320] = {0};
    int blueIntensity[320] = {0};
    int redIntensity[320] = {0};
    int greenIntensity[320] = {0};
    double similarities[4] = {0};

    while (1) {
        int blackcount = 0;
        cv::Mat frame;

        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::Mat colourDisp(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat track(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));

        for (int x = 0; x < 320; x++) {
            for (int y = 0; y < 240; y++) {
                int r = frame.at<cv::Vec3b>(y, x)[2];
                int g = frame.at<cv::Vec3b>(y, x)[1];
                int b = frame.at<cv::Vec3b>(y, x)[0];

                rgb2hsv(r, g, b, &h, &s, &v);

                // Handle purple pixels
                if ((h <= 330 && h >= 270 && v <= 45) || ((h >= 330 || h <= 15) && (s <= 50 || v <= 40)) || (h >= 200 && h <= 280 && (s <= 40 || v <= 30))) {
                    colourDisp.at<cv::Vec3b>(Point(319 - x, 239 - y)) = {255, 255, 255}; // Set purple pixels to white
                    track.at<cv::Vec3b>(Point(319 - x, 239 - y)) = {0, 0, 0};
                } else {
                    colourDisp.at<cv::Vec3b>(Point(319 - x, 239 - y)) = {0, 0, 0}; // Set non-purple pixels to black
                }

                // Handle black pixels
                if (v <= 20) {
                    track.at<cv::Vec3b>(Point(x, y)) = {255, 255, 255};
                    if (y < 100) {
                        blackIntensity[x] += 1;
                        blackcount++;
                    }
                } else if ((h >= 330 || h <= 20) && (s >= 50) && (v >= 35)) { // Handle red pixels
                    track.at<cv::Vec3b>(Point(x, y)) = {0, 0, 255};
                    if (y < 100) {
                        redIntensity[x] += 1;
                    }
                } else if (h <= 280 && h >= 175 && v >= 25 && s >= 50) { // Handle blue pixels
                    track.at<cv::Vec3b>(Point(x, y)) = {255, 0, 0};
                    if (y < 100) {
                        blueIntensity[x] += 1;
                    }
                } else if (h <= 170 && h >= 90 && v >= 15 && s >= 50) { // Handle green pixels
                    track.at<cv::Vec3b>(Point(x, y)) = {0, 255, 0};
                    if (y < 100) {
                        greenIntensity[x] += 1;
                    }
                } else {
                    track.at<cv::Vec3b>(Point(x, y)) = {0, 0, 0};
                }
            }
        }

        cv::imshow("track view", track);

        // Handle further image processing and analysis...

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        if (key == 27) {
            break;
        }
    }

    close(file);
    closeCV(); // Disable the camera and close any windows
