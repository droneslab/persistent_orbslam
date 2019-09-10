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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

//Zakieh
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h>
#include "performance_tools.h"

#include <stdlib.h>
#include <fstream>

#include"../../../include/System.h"

using namespace std;
int count_image = 0;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), 
     delta_time_pause_for_wifi(ros::Duration(10.0)),
     delta_time_move_for_wifi(ros::Duration(3.0)){
        //Zakieh
        collect_wifi = false;
        track_rgbd = false;
        collect_wifi_odom_threshold = 0.0001;
        start = false;
        last_time = ros::Time(0);
        start_time = ros::Time(0);
        finish_time = ros::Time(0);

        last_robot_yaw = 0;
        last_robot_roll = 0;
        last_robot_pitch = 0;
        last_robot_x = 0;  
        last_robot_y = 0;
        cur_robot_x = 0;
        cur_robot_y = 0;
        cur_robot_yaw = 0;
        cur_robot_pitch = 0;
        cur_robot_roll = 0;
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    //Zakieh
    void GrabOdom(const nav_msgs::OdometryConstPtr& msgOdom);


    ORB_SLAM2::System* mpSLAM;

    //Zakieh
    bool collect_wifi;
    bool track_rgbd;
    double collect_wifi_odom_threshold;
    ros::Duration delta_time_pause_for_wifi;
    ros::Duration delta_time_move_for_wifi;
    std::vector<ros::Time> pause_times;
    bool start;
    ros::Time last_time;
    ros::Time start_time;
    ros::Time finish_time;
    double last_robot_yaw;
    double last_robot_roll;
    double last_robot_pitch;
    float last_robot_x;  
    float last_robot_y;
    float cur_robot_x;
    float cur_robot_y;
    double cur_robot_yaw;
    double cur_robot_pitch;
    double cur_robot_roll;

    

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings minRansacMatches distanceForPermanence always_make_keyframe" << endl;        
        ros::shutdown();
        return 1;
    }



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,atoi(argv[3]),atof(argv[4]),atoi(argv[5]));
    
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/head_xtion/rgb/image_color", 100); //"/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/head_xtion/depth/image", 100); // /camera/depth_registered/image_raw", 1);

    //Zakieh
//    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &ImageGrabber::GrabOdom, &igb); 


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
//    ros::spin();
//    std::vector<std::string> folder_names= {"2018-11-07-12-30-34", "2018-11-09-12-32-50", "2018-11-09-15-08-33", "2018-11-12-12-39-54", "2018-11-14-12-38-11", "2018-11-26-14-09-58"}; //"2018-11-09-15-08-33", };//, "2018-11-26-14-09-58"};//, };
//    std::vector<std::string> folder_names= {"08022018_1_vel", "08032018_0830", "08072018_0900", "08092018_0930", "08092018_0945", "08102018_0930"};
    std::vector<std::string> folder_names= {"place_6"}; //, "place_3", "place_6"};
    std::vector<bool> reloc_states= {false, false, false, false, false, false, false, true};//, true};//, };

    int count_line = 0;

    for (unsigned int i = 0; i < folder_names.size(); i++)
    {
        std::cout << folder_names[i] << std::endl;
        std::string file_name = "/home/zakieh/PhD/data/semantic_mapping/NovTrain/" + folder_names[i] + "/rgb-depth/times.txt";
        ifstream myfile (file_name);
        std::string line;
        while ( getline (myfile,line) )
        {
            cv::Mat rgb, depth;
            count_line++;
            file_name = "/home/zakieh/PhD/data/semantic_mapping/NovTrain/" + folder_names[i] + "/rgb-depth/rgb" +line+ ".yml";
            cv::FileStorage fs;
            fs.open(file_name, cv::FileStorage::READ);
            fs["matName"] >> rgb;
            fs.release();

            file_name = "/home/zakieh/PhD/data/semantic_mapping/NovTrain/" + folder_names[i] + "/rgb-depth/depth" +line+ ".yml";
            fs.open(file_name, cv::FileStorage::READ);
            fs["matName"] >> depth;
            fs.release();
            SLAM.TrackRGBD(rgb,depth,atof(line.c_str()),reloc_states[i]);
        //    if (i > 0 && count_line == 5)
        //        break;
            ros::Duration(0.25).sleep();

        }
        myfile.close();
        SLAM.SaveKeyFrameTrajectoryTUM("temp.txt");

    }
    std::cout << "total frames " << count_line << std::endl;

    
    // Stop all threads
    SLAM.Shutdown();
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


    // Save camera trajectory


    ros::shutdown();

    return 0;
        
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    count_image++;
//    if (track_rgbd) 
//    {
        std::cout << "rgbd " << count_image << " in process" << std::endl; 
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        double lastPoseStamp_ = cv_ptrRGB->header.stamp.toSec();
        std::cout << "writing images" << std::endl;
        std::ostringstream s;
        s << std::fixed << "/home/zakieh/PhD/data/semantic_mapping/NovTrain/temp_data/rgb-depth/rgb" << lastPoseStamp_ << ".yml";
        std::string file_name(s.str());
        cv::FileStorage file(file_name, cv::FileStorage::WRITE);
        file << "matName" << cv_ptrRGB->image;

        std::ostringstream s3;
        s3 << std::fixed << "/home/zakieh/PhD/data/semantic_mapping/NovTrain/temp_data/rgb-depth/rgb" << lastPoseStamp_ << ".bmp";
        std::string file_name3(s3.str());
        cv::imwrite(file_name3, cv_ptrRGB->image);

        std::ostringstream s2;
        s2 << std::fixed << "/home/zakieh/PhD/data/semantic_mapping/NovTrain/temp_data/rgb-depth/depth" << lastPoseStamp_ << ".yml";
        std::string file_name2(s2.str());
        cv::FileStorage file2(file_name2, cv::FileStorage::WRITE);
        file2 << "matName" << cv_ptrD->image;
    //    mpSLAM->TrackRGBD(cv_ptrRGB->image,lastPoseStamp_);
//    }
}

void ImageGrabber::GrabOdom(const nav_msgs::OdometryConstPtr& msgOdom)
{
    if (msgOdom) {

        performancetools::Timer timer;
        timer.begin();
        if (start == false){
            finish_time = msgOdom->header.stamp;
            start_time = msgOdom->header.stamp;
        }

        last_time = msgOdom->header.stamp;
        tf::Quaternion q(msgOdom->pose.pose.orientation.x, msgOdom->pose.pose.orientation.y, msgOdom->pose.pose.orientation.z, msgOdom->pose.pose.orientation.w);
        tf::Matrix3x3 m2(q);
        m2.getRPY(cur_robot_roll, cur_robot_pitch, cur_robot_yaw);
        cur_robot_x = msgOdom->pose.pose.position.x;
        cur_robot_y = msgOdom->pose.pose.position.y;
        float robot_displacement = sqrt(pow(cur_robot_x - last_robot_x, 2)+pow(cur_robot_y -last_robot_y, 2));
        double robot_rotation = fabs(cur_robot_roll - last_robot_roll)+fabs(cur_robot_pitch - last_robot_pitch)+fabs(cur_robot_yaw - last_robot_yaw);

        if (robot_displacement < collect_wifi_odom_threshold && robot_rotation < collect_wifi_odom_threshold) 
        { 
           
            track_rgbd = false;
            if (collect_wifi == false || start == false) {
                
                start = true;
                start_time = msgOdom->header.stamp; 
                collect_wifi = true;
            }

        /*    if (msgOdom->header.stamp - start_time > delta_time_pause_for_wifi && count_pause == 25){
                std::cout << "wifi adding node due to more 30 second pause" << std::endl;
                wifi_avg_inf.clear();
                for(std::map<std::string, double>::iterator iter = wifi_signal_strength.begin(); iter != wifi_signal_strength.end(); iter++)
                    wifi_avg_inf[iter->first] = ((float)iter->second)/((float)wifi_ID_freq[iter->first]);
                track_rgbd = true;
                   
            } */   

        }
        else
        {
            
            if (collect_wifi == true || start == false) {

                start = true;
                finish_time = msgOdom->header.stamp;

                collect_wifi = false;

//                std::cout << "track rgbd true" << std::endl;

                // This condition makes sure we have paused for enough duration to get stabilized wifi readings
            /*    if (finish_time - start_time > delta_time_pause_for_wifi) {
     
                    count_pause++;
                    pause_times.push_back(start_time);
                    pause_times.push_back(finish_time);
                   


                    //  the following is for recording similarities between different pauses in text files
                    
                    char buffer2[150];
                    cv::Mat visual_img;
                    cv::Mat depth, depth_mono8_img; 
                    std_msgs::Header depth_header;
                    Node* node_ptr = new NodeWifi(visual_img, depth, depth_mono8_img, cam_infos.back(), depth_header, detector_, extractor_, current_time, wifi_avg_inf);
                    std::cout << "markkk call back nodes size " << nodes.size() << " pausing nodes size " << nodes2.size() << " wifi size " << wifi_avg_inf.size() << std::endl;
                    if (previous_move >= countt) {
                        std::cout << "markkk_bot rotated"  << std::endl;
                    } 
                    if (countt > previous_move)
                        sprintf(buffer2, "/home/zakieh/PhD/wifi-mapping/Data/davis/Praneeth_third/laptop_turtle/unique_MAC_constructed/MSE_intersect_3/pause%d.txt", int(nodes2.size())); 
                    else
                        sprintf(buffer2, "/home/zakieh/PhD/wifi-mapping/Data/davis/Praneeth_third/laptop_turtle/unique_MAC_constructed/MSE_intersect_3/pause%d_rotation.txt", int(nodes2.size())); 
                      
                    myfile.open(buffer2);
                    for (int kk = 0 ; kk < nodes2.size(); kk++){
                        double wifi_value = ((NodeWifi*)node_ptr)->wifiSimilarity((NodeWifi*)nodes2[kk]);
                        myfile << wifi_value << "\n";
                    }
                    nodes2.push_back(node_ptr);
                    myfile.close();

                  
                    
     
                }*/
            }
            track_rgbd = true;

        }

        last_robot_x = cur_robot_x;
        last_robot_y = cur_robot_y;
        last_robot_pitch = cur_robot_pitch;
        last_robot_roll = cur_robot_roll;   
        last_robot_yaw = cur_robot_yaw;

        std::cout << "PERFORMANCE _ GRABODOM _ " << timer.elapsed() << std::endl;
    }
    
}



