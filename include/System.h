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


#ifndef SYSTEM_H
#define SYSTEM_H

#include <ros/ros.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"

namespace orb_tracker
{

class Map;
class Tracking;
class LocalMapping;

class System
{

public:

    // Initialize the SLAM system. It launches the Local Mapping and Loop Closing threads.
    System(const string &strVocFile);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, float &altitude);

    // Set tracker params
    void SetTrackerParams(cv::Mat K, float baseline, bool pub_range);

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

private:

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;


    // System threads: Local Mapping.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
};

}// namespace orb_tracker

#endif // SYSTEM_H
