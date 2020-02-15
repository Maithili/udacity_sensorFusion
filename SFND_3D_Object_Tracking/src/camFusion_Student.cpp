
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(2000); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, BoundingBox &prevBoundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for (std::vector<cv::DMatch>::const_iterator it = kptMatches.begin(); it!=kptMatches.end(); it++)
    {
        if (boundingBox.roi.contains(kptsCurr[it->trainIdx].pt) && prevBoundingBox.roi.contains(kptsPrev[it->queryIdx].pt))
        {
            boundingBox.keypoints.push_back(kptsCurr[it->trainIdx]);
            boundingBox.kptMatches.push_back(*it);
        }
    }
}

float distance(cv::Point2f p1, cv::Point2f p2)
{
    cv::Point2f diff = p1-p2;
    float dist = pow(diff.dot(diff),0.5);
    return (isnan(dist)? -1 : dist);
}

float getDistanceRatio(std::vector<cv::Point2f> &prevPts, std::vector<cv::Point2f> &currPts, int numPts)
{
    std::vector<float> ratios;
    float averageRatio = 0;
    int n = 0;
    for (int i=0 ; i<numPts ; i++)
    {
        for (int j=i+1 ; j<numPts ; j++)
        {
            float d1 = distance(currPts[i],currPts[j]);
            float d2 = distance(prevPts[i],prevPts[j]);
            cout<<d1<<" "<<d2<<endl;
            if ( std::min(d1,d2) < 0.01 )
                continue;
            n++;
            ratios.push_back(d1/d2);
            averageRatio += d1/d2;
        }
    }
    averageRatio /= n;
    std::sort(ratios.begin(), ratios.end());
    for (std::vector<float>::const_iterator it = ratios.begin(); it!=ratios.end(); it++)
    {
        cout << *it << " ";
    }
    cout<<endl<<"distance ratios in image :"<<averageRatio<<endl;
    cout<<endl<<"median distance ratio :"<<ratios[round(n/2)]<<endl;
    return averageRatio;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    std::vector<cv::Point2f> prevMatchedPts;
    std::vector<cv::Point2f> currMatchedPts;
    int numMatches = 0;
    for (std::vector<cv::DMatch>::const_iterator it = kptMatches.begin(); it!=kptMatches.end(); it++)
    {
        if(it->distance < 30)
        {
            prevMatchedPts.push_back(kptsPrev[it->queryIdx].pt);
            currMatchedPts.push_back(kptsCurr[it->trainIdx].pt);
            numMatches ++;
        }
    }
    std::cout<<"Number of matches for camera distance :"<<numMatches<<endl;
    float d_ratio = getDistanceRatio(prevMatchedPts, currMatchedPts, numMatches);
    TTC = (-1/frameRate)/(1-d_ratio);
}

float distanceToLidarCloud(std::vector<LidarPoint> &lidarPoints)
{
    float mean_x = 0;
    float var_x = 0;
    for (std::vector<LidarPoint>::const_iterator it = lidarPoints.begin(); it != lidarPoints.end(); it++)
        mean_x += it->x;
    mean_x /= lidarPoints.size();
    for (std::vector<LidarPoint>::const_iterator it = lidarPoints.begin(); it != lidarPoints.end(); it++)
        var_x += pow(mean_x-it->x,2);
    var_x /= lidarPoints.size();
    return (mean_x-2*pow(var_x,0.5));
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    float currDistance = distanceToLidarCloud(lidarPointsCurr);
    float prevDistance = distanceToLidarCloud(lidarPointsPrev);
    TTC = ( currDistance * (1/frameRate) ) / ( prevDistance - currDistance );
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int bbMatchCountGrid[prevFrame.boundingBoxes.size()+1][currFrame.boundingBoxes.size()+1];
    for (int i = 0; i <= prevFrame.boundingBoxes.size() ; i++)
        for (int j = 0; j <= currFrame.boundingBoxes.size() ; j++)
            bbMatchCountGrid[i][j] = 0;
    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it!=matches.end(); it++)
    {
        auto prevFrameKeypoint = prevFrame.keypoints[it->queryIdx];
        auto currFrameKeypoint = currFrame.keypoints[it->trainIdx];
        int prevFrameBoxidx = 0;
        std::vector<int> prevBoxEnclosingBoxes;
        for (prevFrameBoxidx = 0; prevFrameBoxidx < prevFrame.boundingBoxes.size(); prevFrameBoxidx++)
        {
            if(prevFrame.boundingBoxes[prevFrameBoxidx].roi.contains(prevFrameKeypoint.pt))
            {
                prevBoxEnclosingBoxes.push_back(prevFrameBoxidx);
            }
        }
        int currFrameBoxidx = 0;
        std::vector<int> currBoxEnclosingBoxes;
        for (currFrameBoxidx = 0; currFrameBoxidx < currFrame.boundingBoxes.size(); currFrameBoxidx++)
        {
            if(currFrame.boundingBoxes[currFrameBoxidx].roi.contains(currFrameKeypoint.pt))
            {
                currBoxEnclosingBoxes.push_back(currFrameBoxidx);
            }
        }
        for (int i=0; i<prevBoxEnclosingBoxes.size();i++)
            for(int j=0; j<currBoxEnclosingBoxes.size();j++)
                bbMatchCountGrid[prevBoxEnclosingBoxes[i]][currBoxEnclosingBoxes[j]]++;
    }

    for (int i = 0; i < prevFrame.boundingBoxes.size() ; i++)
    {
        int max_matches = 0;
        int max_index = -1;
        for (int j = 0; j < currFrame.boundingBoxes.size() ; j++)
        {
            if (bbMatchCountGrid[i][j] > max_matches)
            {
                max_matches = bbMatchCountGrid[i][j];
                max_index = j;
            }
            std::cout<<bbMatchCountGrid[i][j]<<" ";
        }
                std::cout<<endl;
        if (max_index == -1)
            continue;
        bbBestMatches.insert({prevFrame.boundingBoxes[i].boxID,currFrame.boundingBoxes[max_index].boxID});
        bool bVis = false;
        if (bVis)
        {
            cv::Mat visImageCurr = currFrame.cameraImg.clone();

            for (int j = 0; j < currFrame.boundingBoxes.size() ; j++)
            {
                int color = int(max(min(float(bbMatchCountGrid[i][j])/float(bbMatchCountGrid[i][max_index])*255.0,255.0),0.0));
                cv::rectangle(visImageCurr, currFrame.boundingBoxes[j].roi, cv::Scalar( 0, 0, color),3);
            }

            cv::Mat visImagePrev = prevFrame.cameraImg.clone();
            cv::rectangle(visImagePrev, prevFrame.boundingBoxes[i].roi, cv::Scalar( 0, 0, 255),3);
            cv::namedWindow("current image", 6);
            imshow("current image", visImageCurr);
            cv::namedWindow("previous image", 7);
            imshow("previous image", visImagePrev);
            cv::waitKey(2000);
        }
    }
}
