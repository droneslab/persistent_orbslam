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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "performance_tools.h"


#include<mutex>
#include<limits>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;
float perm_dist;

//Zaki
MapPoint::MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap, double timestamp, float dist):
    mnFirstKFid(FirstKFid), mnFirstFrame(FirstFrame), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnLastFrameOutlier(0), mnLastFrameFilter(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    
    //zaki_change
    if (dist > 0.0)
        perm_dist = dist;
    overhead_time = 0.0;
    
    update_times.push_back(timestamp);
    update_times_ids.push_back(mnFirstFrame);
    lambda_u = 0.01/3600;
    lambda_l = std::numeric_limits<double>::epsilon(); //0.0001/(24*3600); //std::numeric_limits<double>::epsilon();
    P_M = .4;
    P_F = .01;
    logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);
    filter = new PersistenceFilter(logS_T, timestamp);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
    current_persistence = 1.0;

    //Zakieh
/*    if (mnFirstFrame < 101)
    	video_num = 1;
    else if (mnFirstFrame < 229)
    	video_num = 2;
    else if (mnFirstFrame < 458)
    	video_num = 3;
    else if (mnFirstFrame < 594)
    	video_num = 4;
    else if (mnFirstFrame < 751)
    	video_num = 5;
	else
		video_num = 6;*/

	if (mnFirstFrame < 299)
    	video_num = 1;
    else if (mnFirstFrame < 549)
    	video_num = 2;
    else if (mnFirstFrame < 755)
    	video_num = 3;
    else if (mnFirstFrame < 952)
    	video_num = 4;
    else if (mnFirstFrame < 1036)
    	video_num = 5;
	else
		video_num = 6;

	intra_missed_detection = 0;
    inter_missed_detection = 0;  
    intra_reobservation = 0;
    inter_reobservation = 0;


    
}



MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap, double timestamp, float dist):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnLastFrameOutlier(0), mnLastFrameFilter(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    //zaki_change
    if (dist > 0.0)
        perm_dist = dist;
    overhead_time = 0.0;
    update_times.push_back(timestamp);
    update_times_ids.push_back(mnFirstFrame);


    lambda_u = 0.01/3600;
    lambda_l = std::numeric_limits<double>::epsilon(); //0.0001/(24*3600); //std::numeric_limits<double>::epsilon();
    P_M = .4;
    P_F = .01;
    logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);
    filter = new PersistenceFilter(logS_T, timestamp);


    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
    current_persistence = 1.0;


    //Zakieh
/*    if (mnFirstFrame < 101)
    	video_num = 1;
    else if (mnFirstFrame < 229)
    	video_num = 2;
    else if (mnFirstFrame < 458)
    	video_num = 3;
    else if (mnFirstFrame < 594)
    	video_num = 4;
    else if (mnFirstFrame < 751)
    	video_num = 5;
	else
		video_num = 6;*/

    if (mnFirstFrame < 299)
    	video_num = 1;
    else if (mnFirstFrame < 549)
    	video_num = 2;
    else if (mnFirstFrame < 755)
    	video_num = 3;
    else if (mnFirstFrame < 952)
    	video_num = 4;
    else if (mnFirstFrame < 1036)
    	video_num = 5;
	else
		video_num = 6;

	intra_missed_detection = 0;
    inter_missed_detection = 0;  
    intra_reobservation = 0;
    inter_reobservation = 0;



}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF, double timestamp, float distt):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnLastFrameOutlier(0), mnLastFrameFilter(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    //zaki_change
    if (distt > 0.0)
        perm_dist = distt;
    overhead_time = 0.0;
    update_times.push_back(timestamp);
    update_times_ids.push_back(mnFirstFrame);


    lambda_u = 0.01/3600;
    lambda_l = std::numeric_limits<double>::epsilon(); //0.0001/(24*3600); //std::numeric_limits<double>::epsilon();
    P_M = .4;
    P_F = .01;
    logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);
    filter = new PersistenceFilter(logS_T, timestamp);


    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
    current_persistence = 1.0;


    //Zakieh
/*    if (mnFirstFrame < 101)
    	video_num = 1;
    else if (mnFirstFrame < 229)
    	video_num = 2;
    else if (mnFirstFrame < 458)
    	video_num = 3;
    else if (mnFirstFrame < 594)
    	video_num = 4;
    else if (mnFirstFrame < 751)
    	video_num = 5;
	else
		video_num = 6;*/

    if (mnFirstFrame < 299)
    	video_num = 1;
    else if (mnFirstFrame < 549)
    	video_num = 2;
    else if (mnFirstFrame < 755)
    	video_num = 3;
    else if (mnFirstFrame < 952)
    	video_num = 4;
    else if (mnFirstFrame < 1036)
    	video_num = 5;
	else
		video_num = 6;

	intra_missed_detection = 0;
    inter_missed_detection = 0;  
    intra_reobservation = 0;
    inter_reobservation = 0;

}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::SetReferenceKeyFrame(KeyFrame* pRefKF) {
  mpRefKF = pRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
    {
       // std::cout << "map point " << mnId<< " added to keyframe " << pKF->mnId << " for 2 units" << std::endl;
        nObs+=2;
    }
    else
    {
      //  std::cout << "map point " << mnId<< " added to keyframe " << pKF->mnId << " for 1 units" << std::endl;
        nObs++;
    }
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;
            

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }

    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag(bool delete_keypoint, const char* str)
{
//    std::cout << "map point " << mnId << " is setting to bad from " << str << std::endl;
    
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        
       

    /*    if (pKF->mnId >= 40 && pKF->mnId <= 60)
        {
            std::cout << "erased map point " << mnId << " from " << pKF->mnId << std::endl;
            cv::Mat im;
            cvtColor(pKF->imGray,im,CV_GRAY2BGR);
            cv::circle(im,pKF->mvKeys[mit->second].pt,3,cv::Scalar(0,0,255),-1);
            std::ostringstream s;
            s << std::fixed << "erased" << pKF->mnId << "_" << mnId <<".bmp";
            std::string file_name(s.str());
            cv::imwrite(file_name, im);
        }*/

        pKF->EraseMapPointMatch(mit->second, delete_keypoint);
    }

    mpMap->EraseMapPoint(this);
    std::cout << std::fixed << "overhead_map_point " << mnId << " " << overhead_time << std::endl << std::flush;
    if (delete_keypoint)
    	std::cout << std::fixed << "map_point reobservation " << mnId << " " << mnFirstFrame << " " << video_num << " " << intra_reobservation << " " << intra_missed_detection << " " << inter_reobservation << " " << inter_missed_detection << std::endl << std::flush;
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    //zaki_log
//    std::cout <<  "permanence for the point to be replaced " << std::endl;
//    for (unsigned int i = 0 ; i < update_times.size() ; i++)
//    {
//        std::cout << std::fixed << update_times[i] << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "permanence for the replacing point " << std::endl;
//    for (unsigned int i = 0 ; i < pMP->update_times.size() ; i++)
//    {
//        std::cout << std::fixed << pMP->update_times[i] << " ";
//    }
//    std::cout << std::endl;

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
        /*    cv::Mat im;
            cvtColor(pKF->imGray,im,CV_GRAY2BGR);


            cv::circle(im,pKF->mvKeys[mit->second].pt,3,cv::Scalar(0,0,255),-1);
            std::ostringstream s;
            s << std::fixed << "erased" << pKF->mnId << "_" << mnId <<".bmp";
            std::string file_name(s.str());
            std::cout << "removing map point " << mnId << " from " << pKF->mnId << " file name " << file_name << std::endl;
            for (unsigned int i = 0 ; i < update_times.size(); i++)
            {
                std::cout << update_times_ids[i] << " ";
            }
            std::cout << std::endl;
            cv::circle(im,pKF->mvKeys[pMP->GetIndexInKeyFrame(pKF)].pt,3,cv::Scalar(0,255,0),-1);

            cv::imwrite(file_name, im);*/

            pKF->EraseMapPointMatch(mit->second); //zaki_question
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
    std::cout << std::fixed << "overhead_map_point " << mnId << " " << overhead_time << std::endl << std::flush;
//    std::cout << std::fixed << "map_point reobservation " << mnId << " " << mnFirstFrame << " " << video_num << " " << intra_reobservation << " " << intra_missed_detection << " " << inter_reobservation << " " << inter_missed_detection << std::endl << std::flush;
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

//zaki_change
bool MapPoint::isAvailable(double current_time)
{
    performancetools::Timer timer;
    timer.begin();
   
    {
        unique_lock<mutex> lock(mMutexPos);
        double filter_posterior = filter->predict(current_time);
        if (filter_posterior > 0)  // Compute posterior prediction
            current_persistence = filter_posterior;
    }
    if (current_persistence < 0.2)
    {
    
        
    //    cv::Mat im;
    //    cvtColor(mpRefKF->imGray,im,CV_GRAY2BGR);

    //    cv::circle(im,mpRefKF->mvKeys[mObservations[mpRefKF]].pt,3,cv::Scalar(0,0,255),-1);
    //    std::cout << "erased" << mnId << std::endl;
    //    std::ostringstream s;
    //    s << "erased" << mnId <<".bmp";
    //    std::string file_name(s.str());
    //    cv::imwrite(file_name, im);


    //    for (unsigned int i = 0 ; i < update_times.size(); i++)
    //    {
    //        std::cout << update_times_ids[i] << " ";
    //    }
    //    std::cout << std::endl;
        return false;
    }
    return true;
}
bool MapPoint::updateFilter(double current_time, long unsigned int current_id, bool observe, cv::Mat observer_pose, cv::Mat observer_angle, bool is_occluded) // dist is between pos of map point ref keyframe and pos of current frame or keyframe
{
   // std::cout << std::fixed << "permanence Updating map point " << mnId << " for " << current_id<< " " << observe << std::endl; 
    performancetools::Timer timer;
    timer.begin();
    {
        float dist = -1.0;
        float angle = -1.0;

        if (!observe && !observer_pose.empty() && !is_occluded) //if it is occluded, we don't compute distance and angle
        {
            cv::Mat t = mpRefKF->GetCameraCenter();
            dist = cv::norm(t-observer_pose);

            cv::Mat R = mpRefKF->GetRotation().t();
            Eigen::Matrix<double,3,3> eigMat = Converter::toMatrix3d(R);
            Eigen::Quaterniond qq(eigMat);

            eigMat = Converter::toMatrix3d(observer_angle);
            Eigen::Quaterniond qq2(eigMat);

            angle = qq2.angularDistance(qq)/M_PI*180;

            if (angle > 180)
                angle = 360 - angle;
           
        }

        unique_lock<mutex> lock(mMutexPos);
        if (current_time > fabs(update_times.back()))
        {
            if (observe || (angle < 20 && dist < perm_dist && !is_occluded))
            {
                filter->update(observe, current_time, P_M, P_F);  // Update the filter
                update_times.push_back((observe?1.0:-1.0)*current_time);
                update_times_ids.push_back((observe?1.0:-1.0)*current_id);


                int update_video_num;
            /*    if (current_id < 101)
    				update_video_num = 1;
    			else if (current_id < 229)
			    	update_video_num = 2;
			    else if (current_id < 458)
			    	update_video_num = 3;
			    else if (current_id < 594)
			    	update_video_num = 4;
			    else if (current_id < 751)
			    	update_video_num = 5;
				else
					update_video_num = 6;*/

                if (current_id < 299)
    				update_video_num = 1;
    			else if (current_id < 549)
			    	update_video_num = 2;
			    else if (current_id < 755)
			    	update_video_num = 3;
			    else if (current_id < 952)
			    	update_video_num = 4;
			    else if (current_id < 1036)
			    	update_video_num = 5;
				else
					update_video_num = 6;

				
				if (update_video_num == video_num)
				{
					observe?intra_reobservation++:intra_missed_detection++; 
				}
				else
				{
					observe?inter_reobservation++:inter_missed_detection++; 
				}

            //    if (!observe)
            //    {
            //        std::cout << "distance of " << mnId << " for id " << current_id << " is " << dist << " view point " << mTrackViewCos << std::endl;
             //   }
                overhead_time += timer.elapsed();
                return true;
            }
        }
        else if (update_times.size() > 1 && current_time == fabs(update_times.back()) && ((observe?1.0:-1.0)*current_time) != update_times.back())
        {
        //    std::cout << "reverse updating since we had a different observation at that time" << std::endl;
        //    for (unsigned int i = 0 ; i < update_times.size(); i++)
        //    {
        //        std::cout << update_times[i] << " ";
        //    }
        //    std::cout << std::endl;
            if (filter->update_reverse(current_time))  // Update the filter
            {
                
            	int previous_id = abs(update_times_ids.back());
            	int previous_update_video_num;
            /*	if (previous_id < 101)
    				previous_update_video_num = 1;
    			else if (previous_id < 229)
			    	previous_update_video_num = 2;
			    else if (previous_id < 458)
			    	previous_update_video_num = 3;
			    else if (previous_id < 594)
			    	previous_update_video_num = 4;
			    else if (previous_id < 751)
			    	previous_update_video_num = 5;
				else
					previous_update_video_num = 6;*/

				if (previous_id < 299)
    				previous_update_video_num = 1;
    			else if (previous_id < 549)
			    	previous_update_video_num = 2;
			    else if (previous_id < 755)
			    	previous_update_video_num = 3;
			    else if (previous_id < 952)
			    	previous_update_video_num = 4;
			    else if (previous_id < 1036)
			    	previous_update_video_num = 5;
				else
					previous_update_video_num = 6;

				

				if (previous_update_video_num == video_num)
				{
					update_times_ids.back()>0?intra_reobservation--:intra_missed_detection--; 
				}
				else
				{
					update_times_ids.back()>0?inter_reobservation--:inter_missed_detection--; 
				}


                update_times.pop_back();
                update_times_ids.pop_back();
                if (observe || (angle < 20 && dist < perm_dist && !is_occluded))
                {
                    filter->update(observe, current_time, P_M, P_F);  // Update the filter
                    update_times.push_back((observe?1.0:-1.0)*current_time);
                    update_times_ids.push_back((observe?1.0:-1.0)*current_id);


                    int update_video_num;
                /*    if (current_id < 101)
	    				update_video_num = 1;
	    			else if (current_id < 229)
				    	update_video_num = 2;
				    else if (current_id < 458)
				    	update_video_num = 3;
				    else if (current_id < 594)
				    	update_video_num = 4;
				    else if (current_id < 751)
				    	update_video_num = 5;
					else
						update_video_num = 6;*/

                    if (current_id < 299)
	    				update_video_num = 1;
	    			else if (current_id < 549)
				    	update_video_num = 2;
				    else if (current_id < 755)
				    	update_video_num = 3;
				    else if (current_id < 952)
				    	update_video_num = 4;
				    else if (current_id < 1036)
				    	update_video_num = 5;
					else
						update_video_num = 6;

					if (update_video_num == video_num)
					{
						observe?intra_reobservation++:intra_missed_detection++; 
					}
					else
					{
						observe?inter_reobservation++:inter_missed_detection++; 
					}

                //    if (!observe)
                //    {
                //        std::cout << "distance of " << mnId << " for id " << current_id << " is " << dist << " view point " << mTrackViewCos <<std::endl;
                //    }
                    overhead_time += timer.elapsed();
                    return true;
                }
            }
        }
        overhead_time += timer.elapsed();
        return false;

    }
}


} //namespace ORB_SLAM
