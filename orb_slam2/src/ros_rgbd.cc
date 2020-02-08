/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

#include "/home/l/catkin_ws/src/orb_slam2/include/orb_slam2/include/System.h"

using namespace std;
using namespace cv;

//////////////////////////////// fix path to orb vocabulary and parameter on your computer //////////////////////
string VocabFile = "/home/l/catkin_ws/src/orb_slam2/include/orb_slam2/Vocabulary/ORBvoc.txt";
string ParamsFile = "/home/l/catkin_ws/src/orb_slam2/include/orb_slam2/RGB-D/parameters.yaml";

cv::Mat rgb_img,depth_img;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORB_SLAM_RGBD");
    ros::start();
 
    int tframe;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(VocabFile,ParamsFile,ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);
    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "camera/rgb/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::imageCallback,&igb,_1,_2));
    ros::spin();
		

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::imageCallback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
  try
  {
    int tframe;
    rgb_img = cv_bridge::toCvShare(msgRGB, "8UC3")->image;
    depth_img = cv_bridge::toCvShare(msgD, "16UC1")->image;
    depth_img.convertTo(depth_img,CV_32F);
    mpSLAM->TrackRGBD(rgb_img,depth_img,tframe);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '8UC3'.", msgRGB->encoding.c_str());
  }
}


