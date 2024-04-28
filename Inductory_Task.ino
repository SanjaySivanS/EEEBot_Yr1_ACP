#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Function to isolate objects based on a specified color range
void isolateColor(const Mat& inputImage, const Scalar& lowerRange, const Scalar& upperRange) {
    // Convert input image to HSV color space
    Mat hsvImage;
    cvtColor(inputImage, hsvImage, COLOR_BGR2HSV);

    // Create a mask based on the color range
    Mat mask;
    inRange(hsvImage, lowerRange, upperRange, mask);

    // Apply the mask to extract the object
    Mat result;
    bitwise_and(inputImage, inputImage, result, mask);

    // Display the result (masked image)
    imshow("Object Detected", result);
    waitKey(0);
}

int main() {
    // Load the image
    Mat image = imread("RedCar.bmp");

    if (image.empty()) {
        cerr << "Error: Image not found or unable to load.\n";
        return 1;
    }

    // Display the original image
    imshow("Original Image", image);
    waitKey(0);

    // Isolate blue color
    Scalar lowerBlue = Scalar(100, 50, 50);   // Lower HSV threshold for blue
    Scalar upperBlue = Scalar(140, 255, 255); // Upper HSV threshold for blue
    isolateColor(image, lowerBlue, upperBlue);

    // Isolate red color
    Scalar lowerRed1 = Scalar(0, 50, 50);     // Lower HSV threshold for red (part 1)
    Scalar upperRed1 = Scalar(10, 255, 255);  // Upper HSV threshold for red (part 1)
    Scalar lowerRed2 = Scalar(170, 50, 50);   // Lower HSV threshold for red (part 2)
    Scalar upperRed2 = Scalar(180, 255, 255); // Upper HSV threshold for red (part 2)

    Mat hsvImage;
    cvtColor(image, hsvImage, COLOR_BGR2HSV); // Convert image to HSV

    Mat redMask1, redMask2, redMask;
    inRange(hsvImage, lowerRed1, upperRed1, redMask1);
    inRange(hsvImage, lowerRed2, upperRed2, redMask2);
    redMask = redMask1 | redMask2;
    Mat resultRed;
    bitwise_and(image, image, resultRed, redMask);
    imshow("Red Object Detected", resultRed);
    waitKey(0);

    // Isolate green color
    Scalar lowerGreen = Scalar(35, 50, 50);   // Lower HSV threshold for green
    Scalar upperGreen = Scalar(80, 255, 255); // Upper HSV threshold for green
    isolateColor(image, lowerGreen, upperGreen);

    return 0;
}
