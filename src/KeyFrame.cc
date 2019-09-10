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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>
#include "performance_tools.h"


namespace ORB_SLAM2
{

int countt  = 0;
long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, cv::Mat im):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), 
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    default_persistence(F.default_persistence), available_keypoints(F.available_keypoints), keysToBow(F.keysToBow), keysToFeat(F.keysToFeat),
    mBowVec(F.mBowVec), mBowVec_vanilla(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), wifi_group(-1), lastCheckedPersistenceTime(0.0), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{

    mnId=nNextId++;
    std::cout << "keyframe_ " << mnId << " frame id _ " << mnFrameId << std::endl;

    //zaki_change
    mvpMapPoints_removed = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));


    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);

    //zaki_change
    overhead_time = 0.0;
    im.copyTo(imGray);
    F.im_depth.copyTo(im_depth);
    removed_words = 0;
    removed_so_far = 0;
    total_points = 0;
    //zaki_change
    //std::ostringstream s;
    //s << "keyframe" << mnId <<".bmp";
    //std::string file_name(s.str());
    //cv::imwrite(file_name, im);
    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        keysToBow = vector<std::pair<unsigned int, double> >(N, std::make_pair(0, 0.0));
        keysToFeat = vector<unsigned int>(N, 0);

        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,keysToBow,keysToFeat,mBowVec,mFeatVec,4); //zaki_change
        //zaki_change
        mBowVec_vanilla = mBowVec;
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
    //zaki_change
    //total_points++;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx, bool delete_keypoint)
{
    performancetools::Timer timer;
    timer.begin();
    
    unique_lock<mutex> lock(mMutexFeatures);
  //  long unsigned int xx = mvpMapPoints[idx]->GetReferenceKeyFrame()->mnId;
  //  long unsigned int xx2 = mvpMapPoints[idx]->mnId;

    //zaki_change
    //MapPoint* pMP = mvpMapPoints[idx];
    //total_points--;

    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
    
   /* if (!delete_keypoint)
    {
        if (xx == mnId)
          countt++;
        //cv::Mat im;
        //cvtColor(imGray,im,CV_GRAY2BGR);

        //cv::circle(im,mvKeys[idx].pt,3,cv::Scalar(0,0,255),-1);
        //std::ostringstream s;
        //s << std::fixed << "erased" << mnId << "_" << xx <<".bmp";
        //std::string file_name(s.str());
        //cv::imwrite(file_name, im);
        std::cout << "map point " << xx2 << " getting erased from keyframe " << mnId << std::endl;
    }*/

    //zaki_change
    if (delete_keypoint)
    {
        removed_so_far++;
        available_keypoints[idx] = false;
        //mvpMapPoints_removed[idx] = pMP;
        if (mBowVec.decreaseWeight(keysToBow[idx].first, keysToBow[idx].second))
        {
            removed_words++;
            //std::cout << "word " << keysToBow[idx].first << " removed from " << mnId << std::endl;
            mpKeyFrameDB->erase_single(this, keysToBow[idx].first); //removing it from inverted file
        }
 
    }
    std::cout << std::fixed << "keyframe_erase " << timer.elapsed() << std::endl;
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP, bool delete_keypoint)
{
    performancetools::Timer timer;
    timer.begin();
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
    {
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);

        //zaki_change
        //total_points--;
        if (delete_keypoint)
        {
            removed_so_far++;
            available_keypoints[idx] = false;
          //  mvpMapPoints_removed[idx] = pMP;
            if (mBowVec.decreaseWeight(keysToBow[idx].first, keysToBow[idx].second))
            {
                removed_words++;
                //std::cout << "bug word2 " << keysToBow[idx].first << " removed from " << mnId << std::endl;
                mpKeyFrameDB->erase_single(this, keysToBow[idx].first); //removing it from inverted file  
            }
        }
    }
    std::cout << std::fixed << "keyframe_erase " << timer.elapsed() << std::endl;

}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);

    std::cout << "PERFORMANCE_edge_ " << mnId << " " << pKF->mnId << std::endl;


}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
    std::cout<< std::fixed << "overhead_keyframe " << mnId << " " << overhead_time << std::endl;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

//zaki_change
void KeyFrame::checkPersistence(double current_time, const char* str)
{
    
 /*   if (current_time > lastCheckedPersistenceTime)
    {

        cv::Mat im;
        cvtColor(imGray,im,CV_GRAY2BGR);


        float default_available = default_persistence->predict(current_time);

        total_points = 0;
        int removed = 0;
        
        for (unsigned int i=0; i < mvpMapPoints.size(); i++)
        {
            MapPoint* pMP = mvpMapPoints[i];
            if (pMP && !pMP->isBad())
            {
                if(!pMP->isAvailable(current_time))
                {   
                    removed++;
                    pMP->SetBadFlag(true); //zaki_change (for removing corresponding keypoint)
                    cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,0,255),-1);

                }
                else
                {
                    total_points++;
                }
            }
            else if(default_available < 0.2)
            {
                removed_so_far++;
                available_keypoints[i] = false;
                std::cout << std::fixed << "deleting keypoint from frame " << mnFrameId << " keyframe " << mnId << " at " << current_time << std::endl;
                if (mBowVec.decreaseWeight(keysToBow[i].first, keysToBow[i].second))
                {
                    removed_words++;
                    //std::cout << "bug word3 " << keysToBow[i].first << " removed from " << mnId << std::endl;

                    mpKeyFrameDB->erase_single(this, keysToBow[i].first); //removing it from inverted file  
                }
                cv::circle(im,mvKeys[i].pt,3,cv::Scalar(255,0,0),-1);
                removed++;


            }
        }
        lastCheckedPersistenceTime = current_time;
        /*if (removed > 0)
        {
            std::ostringstream s;
            s << std::fixed << "removed" << mnId << "-" << mnFrameId << "-" << current_time <<".bmp";
            std::string file_name(s.str());
            cv::imwrite(file_name, im);
        //    std::cout << "removed " << removed << " point from a total of " << count_total << " points" << std::endl;

        }*/

//    }
}
bool KeyFrame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    performancetools::Timer timer;
    timer.begin();

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = GetRotation()*P+GetTranslation();
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        overhead_time+=timer.elapsed();
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
    {
        overhead_time+=timer.elapsed();
        return false;
    }
    if(v<mnMinY || v>mnMaxY)
    {
        overhead_time+=timer.elapsed();
        return false;
    }

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-GetCameraCenter();
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
    {
        overhead_time+=timer.elapsed();
        return false;
    }

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
    {
        overhead_time+=timer.elapsed();
        return false;
    }

    pMP->mTrackViewCosKeyframe = viewCos;
    pMP->mTrackProjX_keyframe = u;
    pMP->mTrackProjY_keyframe = v;
    overhead_time+=timer.elapsed();
 
    return true;
}
void KeyFrame::draw(long unsigned int frame_id)
{
    cv::Mat im;
    cvtColor(imGray,im,CV_GRAY2BGR); 

    for (int i = 0 ; i < N ; i++)
    {
        if (available_keypoints[i] == false)
            cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,0,255),-1);
        else if (mvpMapPoints[i] && !(mvpMapPoints[i]->isBad()))
        { 
            if (mvpMapPoints[i]->current_persistence >= 0.8)           
                cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,255,0),-1);
            else if (mvpMapPoints[i]->current_persistence >= 0.6)
                cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,255,127),-1);
            else if (mvpMapPoints[i]->current_persistence >= 0.4)
                cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,255,255),-1);
            else if (mvpMapPoints[i]->current_persistence >= 0.2)
                cv::circle(im,mvKeys[i].pt,3,cv::Scalar(0,127,255),-1);

        }

    }

    std::ostringstream s;
    s << std::fixed << "removed" << frame_id << "-" << mnId << ".bmp";
    std::string file_name(s.str());
    cv::imwrite(file_name, im);
}

void KeyFrame::getPersistentWords()
{
    perm_words.clear();
  //  perm_levels.clear();
    for (unsigned int i=0; i < mvpMapPoints.size(); i++)
    {
        if (mvpMapPoints[i] && !mvpMapPoints[i]->isBad())
        {
            if (mvpMapPoints[i]->current_persistence >= 0.8)
            {
                perm_words.insert(keysToBow[i].first);
                mBowVec_perm.addWeight(keysToBow[i].first, keysToBow[i].second);

            //    perm_levels.insert(keysToFeat[i]);
            }
        }
    }
    
}

void KeyFrame::getPersistentRelocWords()
{
    reloc_perm_words.clear();
    for (unsigned int i=0; i < mvpMapPoints.size(); i++)
    {
        if (mvpMapPoints[i] && !mvpMapPoints[i]->isBad())
        {
            if (mvpMapPoints[i]->current_persistence >= 0.8)
            {
                reloc_perm_words.insert(keysToBow[i].first);
                mBowVec_reloc_perm.addWeight(keysToBow[i].first, keysToBow[i].second);

            }
        }
    }
    
}

bool KeyFrame::isMapPointVisible(float u, float v, MapPoint* pMP)
{
    performancetools::Timer timer;
    timer.begin();
    //zaki_check
    if (u < 0)
    {
        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = GetRotation()*p3Dw + GetTranslation();

                    // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
        {
            overhead_time+=timer.elapsed();
            return false;
        }

        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        u = fx*x+cx;
        v = fy*y+cy;

        if(!IsInImage(u,v))
        {
            overhead_time+=timer.elapsed();
            return false;
        }
    }
    float d = 0.0;
    int count_pixel = 0;
    for (int ii = u - 1; ii <= u+1; ii++)
    {
        for (int jj = v - 1; jj <= v+1; jj++)
        {

            if (ii >= 0 && jj >= 0 && ii < mnMaxX &&  jj < mnMaxY)
            {
            
                float d2 = im_depth.at<float>(jj,ii);
                if (d2 > 0)
                {
                    d += d2;
                    count_pixel++;
                }
            }
        }

    }
    cv::Mat P = pMP->GetWorldPos(); 
    float distance = cv::norm(P-Ow);
    if (count_pixel > 0)
    {
        if ((d/count_pixel) < (distance-0.3)) //we belive its is occluded by some other object
        {
            overhead_time+=timer.elapsed();
            std::cout << "not visible" << std::endl;
            return false;
        }
    }
    overhead_time+=timer.elapsed();
    return true;
}

} //namespace ORB_SLAM
