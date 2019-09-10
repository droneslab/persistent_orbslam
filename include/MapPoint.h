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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include"persistence_filter.h"
#include"persistence_filter_utils.h"

#include <functional>
#include <iostream>
#include <gsl/gsl_sf_exp.h>


#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:

    MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap, double timestamp, float dist=0.0);


    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap, double timestamp, float dist=0.0);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF, double timestamp, float dist=0.0);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag(bool delete_keypoint=false, const char* str = __builtin_FUNCTION());
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    //Zaki
    void SetReferenceKeyFrame(KeyFrame* pRefKF); //For saving and loading map
    bool isAvailable(double current_time);
    bool updateFilter(double current_time, long unsigned int current_id, bool observe, cv::Mat observed_pose=cv::Mat(), cv::Mat observed_angle=cv::Mat(), bool is_occluded=false);
    void reverseUpdate(double time);



public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    float mTrackViewCosKeyframe; //zaki_change
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    //zaki_change
    long unsigned int mnLastFrameOutlier;// this is for keeping being an outlier at some stage in tracking.
    long unsigned int mnLastFrameFilter;
    long unsigned int mnLastFrameCountObserve; //this is for not loosing features due to unexpected behaviours in operations
    long unsigned int mnLastKeyframeCountObserve; 
    double current_persistence;
    float mTrackProjX_keyframe;
    float mTrackProjY_keyframe;
    double overhead_time;
    int video_num; //for counting inter and intra video reobservations and missed detections
    int intra_missed_detection;
    int inter_missed_detection;  
    int intra_reobservation;
    int inter_reobservation;

    std::vector<double> update_times;
    std::vector<long int> update_times_ids;

    long unsigned int visualSituation;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;



    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;

     //zaki_change
     double lambda_u;
     double lambda_l;
     double P_M;
     double P_F;
     std::function<double(double)> logS_T;
     PersistenceFilter* filter;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
