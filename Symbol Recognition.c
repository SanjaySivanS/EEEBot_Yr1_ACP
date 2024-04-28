#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "opencv_aee.hpp"
#include "main.hpp"
#include "pi2c.h"
#include <iostream>
#include <math.h>

Pi2c car(0x22);

void setup(void) {
    setupCamera(320, 240);
}

bool isNumber(float x) {
    return (x == x);
}

double calculateMSE(cv::Mat image1, cv::Mat image2) {
    if (image1.size() != image2.size()) {
        printf("images have different size\n");
        return -1;
    }
    if (image1.channels() != image2.channels()) {
        printf("Error: Images have different number of channels\n");
        printf("camera image channels: %d\n", image1.channels());
        printf("png channels: %d\n", image2.channels());
        return -1;
    }
    cv::Mat diff;
    cv::absdiff(image1, image2, diff);
    diff.convertTo(diff, CV_32F);
    diff = diff.mul(diff);
    double mse = cv::mean(diff)[0];
    return mse;
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
    setup();
    cv::Mat umbrella = cv::imread("Umbrella.png", cv::IMREAD_GRAYSCALE);
    cv::Mat triangle = cv::imread("Triangle.png", cv::IMREAD_GRAYSCALE);
    cv::Mat circle = cv::imread("Circle.png", cv::IMREAD_GRAYSCALE);
    cv::Mat star = cv::imread("Star.png", cv::IMREAD_GRAYSCALE);
    int file;
    bool followblack = true;
    const char* filename = "/dev/i2c-1";
    int address = 0x08;
    file = open(filename, O_RDWR);
    ioctl(file, I2C_SLAVE, address);
    cv::namedWindow("Photo");
    cv::namedWindow("Colour view: Pink");
    int r, g, b, x = 0, y = 0, h, s, v;
    char symbol;
    float Xpk = 0, denominator = 0, numerator = 0;
    int blackIntensity[320] = {0};
    int blackcount;
    int blueIntensity[320] = {0};
    int redIntensity[320] = {0};
    int greenIntensity[320] = {0};
    double similarities[4] = {0};

    while (1) {
        blackcount = 0;
        cv::Mat frame;
        while (frame.empty())
            frame = captureFrame();
        cv::Mat colourDisp(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat track(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));

        for (int g = 0; g < 320; g++) {
            blackIntensity[g] = 0;
            blueIntensity[g] = 0;
            redIntensity[g] = 0;
            greenIntensity[g] = 0;
        }

        for (int g = 0; g < 4; g++) {
            similarities[g] = 0;
        }

        for (x = 0; x < 320; x++) {
            for (y = 0; y < 240; y++) {
                r = frame.at<cv::Vec3b>(y, x)[2];
                g = frame.at<cv::Vec3b>(y, x)[1];
                b = frame.at<cv::Vec3b>(y, x)[0];
                rgb2hsv(r, g, b, &h, &s, &v);

                if (h <= 330 && h >= 270 && v <= 45 || (h >= 330 || h <= 15) && (s <= 50 || v <= 40) || (h >= 200 && h <= 280 && (s <= 40 || v <= 30))) {
                    colourDisp.at<Vec3b>(Point(319 - x, 239 - y)) = {255, 255, 255};
                    track.at<Vec3b>(Point(319 - x, 239 - y)) = {0, 0, 0};
                } else {
                    colourDisp.at<Vec3b>(Point(319 - x, 239 - y)) = {0, 0, 0};
                }

                if (v <= 20) {
                    track.at<Vec3b>(Point(x, y)) = {255, 255, 255};
                    if (y < 100) {
                        blackIntensity[x] += 1;
                        blackcount++;
                    }
                } else if ((h >= 330 || h <= 20) && (s >= 50) && (v >= 35)) {
                    track.at<Vec3b>(Point(x, y)) = {0, 0, 255};
                    if (y < 100) {
                        redIntensity[x] += 1;
                    }
                } else if (h <= 280 && h >= 175 && v >= 25 && s >= 50) {
                    track.at<Vec3b>(Point(x, y)) = {255, 0, 0};
                    if (y < 100) {
                        blueIntensity[x] += 1;
                    }
                } else if (h <= 170 && h >= 90 && v >= 15 && s >= 50) {
                    track.at<Vec3b>(Point(x, y)) = {0, 255, 0};
                    if (y < 100) {
                        greenIntensity[x] += 1;
                    }
                } else {
                    track.at<Vec3b>(Point(x, y)) = {0, 0, 0};
                }
            }
        }

        cv::imshow("track view", track);
        cv::Mat gray;
        cv::cvtColor(colourDisp, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binaryImage;
        cv::threshold(gray, binaryImage, 64, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binaryImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> approxedcontours(contours.size());

        for (int i = 0; i < contours.size(); i++) {
            cv::approxPolyDP(contours[i], approxedcontours[i], 30, true);
        }

        double maxArea = 0;
        int maxAreaContourIndex = -1;

        for (size_t i = 0; i < approxedcontours.size(); i++) {
            double area = cv::contourArea(approxedcontours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxAreaContourIndex = i;
            }
        }

        printf("Black pixels: %d\n", blackcount);

        if (maxAreaContourIndex != -1) {
            std::vector<cv::Point> largestContour = approxedcontours[maxAreaContourIndex];
            cv::Mat perspectiveMatrix;
            std::vector<cv::Point2f> transformedContour;
            perspectiveMatrix = transformPerspective(largestContour, colourDisp, 350, 350);
            cv::Mat grayImage;

            if (perspectiveMatrix.rows > 0 && perspectiveMatrix.cols > 0) {
                cv::imshow("perspective matrix", perspectiveMatrix);
                cv::cvtColor(perspectiveMatrix, grayImage, cv::COLOR_BGR2GRAY);
            }

            similarities[0] = calculateMSE(grayImage, umbrella);
            similarities[1] = calculateMSE(grayImage, circle);
            similarities[2] = calculateMSE(grayImage, star);
            similarities[3] = calculateMSE(grayImage, triangle);

            double maxVal = 0;
            int maxPos = 0;

            for (int i = 0; i < 4; i++) {
                if (similarities[i] > maxVal) {
                    maxVal = similarities[i];
                    maxPos = i;
                }
            }

            if (maxVal > 20000) {
                followblack = false;
                if (maxPos == 0) {
                    symbol = 'u';
                } else if (maxPos == 1) {
                    symbol = 'c';
                } else if (maxPos == 2) {
                    symbol = 's';
                } else if (maxPos == 3) {
                    symbol = 't';
                }
            }
        }

        Xpk = 0.0f;
        denominator = 0.0f;
        numerator = 0.0f;

        if (followblack == true) {
            symbol = 'e';
            for (int i = 0; i < 320; i++) {
                denominator += blackIntensity[i] * (i - 98);
                numerator += blackIntensity[i];
            }
        } else if (symbol == 's') {
            for (int i = 0; i < 320; i++) {
                denominator += greenIntensity[i] * (i - 98);
                numerator += greenIntensity[i];
            }
        } else if (symbol == 'c') {
            for (int i = 0; i < 320; i++) {
                denominator += redIntensity[i] * (i - 98);
                numerator += redIntensity[i];
            }
        } else if (symbol == 't') {
            for (int i = 0; i < 320; i++) {
                denominator += blueIntensity[i] * (i - 98);
                numerator += blueIntensity[i];
            }
        }

        Xpk = denominator / numerator;

        if (!isNumber(Xpk) && followblack == false) {
            followblack = true;
        }

        if (Xpk < -120) {
            Xpk = -120;
        }

        if (Xpk > 120) {
            Xpk = 120;
        }

        printf("Xpk = %f\n", Xpk);

        if (isNumber(Xpk)) {
            unsigned char buf[sizeof(float)];
            memcpy(buf, &Xpk, sizeof(float));
            write(file, buf, sizeof(float));
        } else {
            followblack = true;
        }

        cv::imshow("Photo", frame);
        cv::imshow("Colour view: Pink", colourDisp);

        int key = cv::waitKey(1);
        key = (key == 255) ? -1 : key;

        if (key == 27) {
            break;
        }
    }

    close(file);
    closeCV();

    return 0;
}
