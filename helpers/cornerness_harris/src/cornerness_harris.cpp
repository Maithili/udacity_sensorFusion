#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

//    // visualize results
//    string windowName = "Harris Corner Detector Response Matrix";
//    cv::namedWindow(windowName, 4);
//    cv::imshow(windowName, dst_norm_scaled);
//    cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix
    // and perform a non-maximum suppression (NMS) in a local neighborhood around
    // each maximum. The resulting coordinates shall be stored in a list of keypoints
    // of the type `vector<cv::KeyPoint>`.
    vector<cv::KeyPoint> HarrisCorners;
    double min_r, max_r = minResponse+1;
    int min_point[2], max_point[2];
    cv::Mat keyImg = dst_norm_scaled;
    while (max_r > minResponse)
    {
        cv::minMaxIdx(dst_norm, &min_r, &max_r, &min_point[0], &max_point[0]);
        int r =5;
        cv::KeyPoint new_keypoint(max_point[1],max_point[0],blockSize);
        HarrisCorners.push_back(new_keypoint);
        cv::Range col_Range(std::max(max_point[0]-r,0), std::min(max_point[0]+r,img.size().height));
        cv::Range row_Range(std::max(max_point[1]-r,0), std::min(max_point[1]+r,img.size().width));
        dst_norm(cv::Rect_<int>(row_Range.start,col_Range.start,row_Range.size(),col_Range.size())) = 0.0;
    }
    drawKeypoints( keyImg, HarrisCorners, keyImg);
    string windowName = "Harris Corner Detector Keypoints";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, keyImg);
    cv::waitKey(0);
}

int main()
{
    cornernessHarris();
}
