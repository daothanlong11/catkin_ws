#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#define RGBD_PATH_SEQUENCE "/home/l/catkin_ws/src/mastering_ros_camera_publish/include/mastering_ros_camera_publish/RGB-D/rgbd_dataset_freiburg1_xyz/"
using namespace std;
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{

    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
     if (fAssociation.is_open())
		std::cout << "OKAYYYYY";
     while(!fAssociation.is_open())
	{
    	// show message:
    	std::cout << "Error opening file";
  	}
	cerr << endl << "OK1" << endl;
    while(!fAssociation.eof())
    {
	
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
cerr << endl << string(sRGB)<< endl << string(sD) << endl;	
        }

    }
}
int main(int argc, char **argv)
{
//Topic ROS
	ros::init(argc, argv,"image_publisher");
	ros::NodeHandle nh1;
	image_transport::ImageTransport it1(nh1);
	image_transport::Publisher pub1 = it1.advertise("camera/rgb/image", 1);
	ros::NodeHandle nh2;
	image_transport::ImageTransport it2(nh2);
	image_transport::Publisher pub2 = it2.advertise("camera/depth/image", 1);
	
// File Path
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps;
	string strAssociationFilename = string("/home/l/Documents/slam_nckh/ORB_SLAM2/Examples/RGB-D/associations/fr1_xyz.txt");
	LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
	int nImages = vstrImageFilenamesRGB.size();
    	if(vstrImageFilenamesRGB.empty())
    	{
        	cerr << endl << "No images found in provided path." << endl;
        	return 1;
    	}
    	else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    	{
        	cerr << endl << "Different number of images for rgb and depth." << endl;
        	return 1;
    	}
//RGB Images and Equivalent Depth Images
	cv::Mat imRGB, imD;
while (nh1.ok())
	{
		for(int ni=0; ni<nImages; ni++)
    		{
			ros::Rate loop_rate(10);		
 			imRGB = cv::imread(string(RGBD_PATH_SEQUENCE)+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
			imD = cv::imread(string(RGBD_PATH_SEQUENCE)+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
			
			sensor_msgs::ImagePtr msgrgb = cv_bridge::CvImage(std_msgs::Header(), "8UC3", imRGB).toImageMsg();
			sensor_msgs::ImagePtr msgd = cv_bridge::CvImage(std_msgs::Header(), "16UC1", imD).toImageMsg();
			pub1.publish(msgrgb);
			pub2.publish(msgd);
			ros::spinOnce();
			loop_rate.sleep();
		}
cerr << endl << "OK67" << endl;
	}
		return 0;

}



