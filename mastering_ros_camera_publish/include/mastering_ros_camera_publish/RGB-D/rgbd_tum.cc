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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/opencv.hpp>
//#include<opencv2/core/core.hpp>
//#include "opencv2/cudaimgproc.hpp"

#include<System.h>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <unistd.h>


using namespace std;
using namespace cv;


int main()
{
    int tframe;
    double fps;
    time_t start, end;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    pipe.start(cfg);
    
    string orb_vocfile = "ORBvoc.txt";
    string parameters_file = "parameters.yaml";
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(orb_vocfile,parameters_file,ORB_SLAM2::System::RGBD,true);
    SLAM.fps_s = &fps;
    rs2::frame_queue postprocessed_frames;
    std::atomic_bool alive{ true };

    std::thread video_processing_thread([&]() {
        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset data;
            if (pipe.poll_for_frames(&data))
            {
                // First make the frames spatially aligned
                data = data.apply_filter(align_to);
                // Send resulting frames for visualization in the main thread
                postprocessed_frames.enqueue(data);
            }
        }
    });

    rs2::frameset current_frameset;
    cv::Mat imRGB, imD;
    //cv::cuda::GpuMat in, out;
    tframe = 0;
    time(&start);
    // Main loop
    
    while(SLAM.stopLoop==false)
    {
        
        postprocessed_frames.poll_for_frame(&current_frameset);
        if (current_frameset)
        {
            rs2::video_frame color_frame = current_frameset.get_color_frame();
            rs2::depth_frame depth_frame = current_frameset.get_depth_frame();

            Mat imRGB(Size(640,480),CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
            Mat imD(Size(640,480),CV_16UC1,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
            imD.convertTo(imD,CV_32F);
            if(imRGB.empty())
            {
                cout<<"Failed to load image!!!"<<endl;
                return 1;
            }

            SLAM.TrackRGBD(imRGB,imD,tframe); 

            if (tframe !=0 && tframe%10==0)
            {
                time(&end);
                double seconds = difftime (end, start);
                fps  = tframe / seconds;
                
            }
            
            
            tframe++;
            
            
        }
       
        
    }
    alive = false;
    video_processing_thread.join();
    // Stop all threads
    SLAM.Shutdown();

    
    time(&end);
    double seconds = difftime (end, start);
    fps  = tframe / seconds;
    cout<<"fps: "<<fps<<endl;
    
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

