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



#include "System.h"
#include "Converter.h"
#include <thread>

namespace orb_tracker
{

System::System(const string &strVocFile) : mbReset(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    // Load ORB Vocabulary
    mpVocabulary = new ORBVocabulary();
    cout << endl << "[orb_tracker]: Loading ORB Vocabulary. This could take a while..." << endl;
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "[orb_tracker]: Wrong path to vocabulary. " << endl;
        cerr << "[orb_tracker]: Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "[orb_tracker]: Vocabulary loaded!" << endl << endl;

    //Create the Map
    mpMap = new Map();

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(mpVocabulary, mpMap);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap);
    mptLocalMapping = new thread(&orb_tracker::LocalMapping::Run,mpLocalMapper);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);
}

void System::SetTrackerParams(cv::Mat K, float baseline, bool pub_range)
{
    mpTracker->SetParams(K, baseline, pub_range);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, float &altitude)
{
    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp,altitude);
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished())
    {
        usleep(5000);
    }
}

} //namespace orb_tracker
