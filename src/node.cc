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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Range.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Converter.h"

using namespace std;

ros::Publisher pose_pub_;
ros::Publisher twist_pub_;
ros::Publisher range_pub_;

static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.02, 0, 0, 0, 0, 0,
    0, 0.02, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };

class ImageGrabber
{
public:
    ImageGrabber(orb_tracker::System* system):pub_range_(true),system_(system),is_init_(false),is_pose_init_(false){}

    void grabStereo(const sensor_msgs::ImageConstPtr& img_left_rect,
                    const sensor_msgs::ImageConstPtr& img_right_rect,
                    const sensor_msgs::CameraInfoConstPtr& img_left_info,
                    const sensor_msgs::CameraInfoConstPtr& img_right_info);

    tf::Transform integrated_pose_;
    bool pub_range_;
    double min_range_;
    double max_range_;

private:

    cv::Mat getCameraModel(sensor_msgs::CameraInfo l_info_msg,
                           sensor_msgs::CameraInfo r_info_msg);

    orb_tracker::System* system_;
    bool is_init_;
    bool is_pose_init_;
    tf::Transform prev_camera_pose_;
    ros::Time prev_stamp_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_tracker");
    ros::start();

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun orb_tracker orb_tracker path_to_vocabulary" << endl;
        ros::shutdown();
        return 1;
    }

    // Create tracker system. It initializes all system threads and gets ready to process frames.
    orb_tracker::System system(argv[1]);

    ImageGrabber igb(&system);

    // Init the integrated pose
    igb.integrated_pose_.setIdentity();

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Get params
    nhp.param("pub_range", igb.pub_range_, true);
    nhp.param("min_range", igb.min_range_, 1.2);
    nhp.param("max_range", igb.max_range_, 6.0);

    // Advertise
    pose_pub_ = nhp.advertise<geometry_msgs::PoseStamped>("visual_odometry_pose", 1);
    twist_pub_ = nhp.advertise<geometry_msgs::TwistWithCovarianceStamped>("visual_odometry_twist", 1);
    if (igb.pub_range_)
        range_pub_ = nhp.advertise<sensor_msgs::Range>("altitude", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_rect_sub(nh, "/camera/left/image_rect", 2);
    message_filters::Subscriber<sensor_msgs::Image> right_rect_sub(nh, "/camera/right/image_rect", 2);
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub(nh, "/camera/left/camera_info", 2);
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub(nh, "/camera/right/camera_info", 2);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                             sensor_msgs::Image,
                                                             sensor_msgs::CameraInfo,
                                                             sensor_msgs::CameraInfo> sync_pol1;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(10), left_rect_sub, right_rect_sub, left_info_sub, right_info_sub);
    sync1.registerCallback(boost::bind(&ImageGrabber::grabStereo,&igb,_1,_2,_3,_4));

    ros::spin();

    // Stop all threads
    system.Shutdown();

    ros::shutdown();

    return 0;
}

cv::Mat ImageGrabber::getCameraModel(sensor_msgs::CameraInfo l_info_msg,
                                     sensor_msgs::CameraInfo r_info_msg)
{
    // Get the binning factors
    int binning_x = l_info_msg.binning_x;
    int binning_y = l_info_msg.binning_y;

    // Get the projection/camera matrix
    const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg.P.data()));
    cv::Mat camera_matrix = P.colRange(cv::Range(0,3)).clone();
    camera_matrix.convertTo(camera_matrix, CV_32F);

    // Are the images scaled?
    if (binning_x > 1 || binning_y > 1)
    {
        camera_matrix.at<float>(0,0) = camera_matrix.at<float>(0,0) / binning_x;
        camera_matrix.at<float>(0,2) = camera_matrix.at<float>(0,2) / binning_x;
        camera_matrix.at<float>(1,1) = camera_matrix.at<float>(1,1) / binning_y;
        camera_matrix.at<float>(1,2) = camera_matrix.at<float>(1,2) / binning_y;
    }
    return camera_matrix;
}

void ImageGrabber::grabStereo(const sensor_msgs::ImageConstPtr& img_left_rect,
                              const sensor_msgs::ImageConstPtr& img_right_rect,
                              const sensor_msgs::CameraInfoConstPtr& img_left_info,
                              const sensor_msgs::CameraInfoConstPtr& img_right_info)
{
    // Initialize if not
    if (!is_init_)
    {
        // Extract camera information
        cv::Mat camera_matrix = getCameraModel(*img_left_info, *img_right_info);
        image_geometry::StereoCameraModel stereo_camera_model;
        stereo_camera_model.fromCameraInfo(*img_left_info, *img_right_info);

        // Stereo baseline times fx
        float baseline = (float)stereo_camera_model.baseline()*camera_matrix.at<float>(0,0);

        // Set params
        system_->SetTrackerParams(camera_matrix, baseline, pub_range_);
        is_init_ = true;
    }

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr_left, cv_ptr_right;
    try
    {
        cv_ptr_left  = cv_bridge::toCvShare(img_left_rect);
        cv_ptr_right = cv_bridge::toCvShare(img_right_rect);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Compute camera pose
    float altitude = -1.0;
    cv::Mat Tcw = system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image, cv_ptr_left->header.stamp.toSec(), altitude);

    // When system got lost, correct with the input odometry (if any)
    if (Tcw.rows == 0 || Tcw.cols == 0)
    {
        // System got lost
        is_pose_init_ = false;

        // Publish an invalid altitude
        sensor_msgs::Range range_msg;
        range_msg.header = cv_ptr_left->header;
        range_msg.min_range = min_range_;
        range_msg.max_range = max_range_;
        range_msg.field_of_view = 60.0/180.0*M_PI;
        range_msg.range = -1.0;
        range_pub_.publish(range_msg);
    }
    else
    {
        // Convert to TF
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        vector<float> qwc = orb_tracker::Converter::toQuaternion(Rwc);
        tf::Quaternion q(qwc[0],qwc[1],qwc[2],qwc[3]);
        tf::Vector3 t(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
        tf::Transform camera_pose(q,t);

        // Is this the first orb_tracker pose? or recovering from a lost?
        if (!is_pose_init_)
        {
            // Yes, this is the first position provided by orb_tracker: store and exit.
            prev_camera_pose_ = camera_pose;
            prev_stamp_ = cv_ptr_left->header.stamp;
            is_pose_init_ = true;

            // Publish the pose only
            if (pose_pub_.getNumSubscribers() > 0)
            {
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header = cv_ptr_left->header;
                tf::poseTFToMsg(integrated_pose_, pose_msg.pose);
                pose_pub_.publish(pose_msg);
            }
            return;
        }
        else
        {
            // Orb_tracker is working
            tf::Transform delta_transform = prev_camera_pose_.inverse() * camera_pose;
            integrated_pose_ *= delta_transform;

            // Get delta time
            ros::Time cur_stamp = cv_ptr_left->header.stamp;
            double delta_t = cur_stamp.toSec() - prev_stamp_.toSec();

            // Create the messages
            if (twist_pub_.getNumSubscribers() > 0)
            {
                geometry_msgs::TwistWithCovarianceStamped twist_msg;
                twist_msg.header.stamp = cv_ptr_left->header.stamp;
                twist_msg.header.frame_id = cv_ptr_left->header.frame_id;
                twist_msg.twist.twist.linear.x = delta_transform.getOrigin().getX() / delta_t;
                twist_msg.twist.twist.linear.y = delta_transform.getOrigin().getY() / delta_t;
                twist_msg.twist.twist.linear.z = delta_transform.getOrigin().getZ() / delta_t;
                tf::Quaternion delta_rot = delta_transform.getRotation();
                tfScalar angle = delta_rot.getAngle();
                tf::Vector3 axis = delta_rot.getAxis();
                tf::Vector3 angular_twist = axis * angle / delta_t;
                twist_msg.twist.twist.angular.x = angular_twist.x();
                twist_msg.twist.twist.angular.y = angular_twist.y();
                twist_msg.twist.twist.angular.z = angular_twist.z();
                twist_msg.twist.covariance = STANDARD_TWIST_COVARIANCE;
                twist_pub_.publish(twist_msg);
            }
            if (pose_pub_.getNumSubscribers() > 0)
            {
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = cv_ptr_left->header.stamp;
                pose_msg.header.frame_id = cv_ptr_left->header.frame_id;
                tf::poseTFToMsg(integrated_pose_, pose_msg.pose);
                pose_pub_.publish(pose_msg);
            }

            // Publish altitude
            if (pub_range_ && range_pub_.getNumSubscribers() > 0)
            {
                // Sanity checks
                if (altitude < min_range_ || altitude > max_range_)
                    altitude = -1.0;

                sensor_msgs::Range range_msg;
                range_msg.header = cv_ptr_left->header;
                range_msg.min_range = min_range_;
                range_msg.max_range = max_range_;
                range_msg.field_of_view = 60.0/180.0*M_PI;
                range_msg.range = altitude;
                range_pub_.publish(range_msg);
            }

            // Store
            prev_camera_pose_ = camera_pose;
            prev_stamp_ = cur_stamp;
        }
    }
}
