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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "performance_tools.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());

    //zaki_change
    mvInvertedFile_vanilla.resize(voc.size());
}


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        mvInvertedFile[vit->first].push_back(pKF);

        //zaki_change
        mvInvertedFile_vanilla[vit->first].push_back(pKF);

    }
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    performancetools::Timer timer;
    timer.begin();

    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }

        //zaki_change for now we assume keyframe culling is not affected by persistence, although it is somewhat.
        list<KeyFrame*> &lKFs2 =   mvInvertedFile_vanilla[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs2.begin(), lend= lKFs2.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs2.erase(lit);
                break;
            }
        }
    }



}

void KeyFrameDatabase::erase_single(KeyFrame* pKF, unsigned int word_id)
{
    unique_lock<mutex> lock(mMutex);


    // List of keyframes that share the word
    list<KeyFrame*> &lKFs =  mvInvertedFile[word_id];

    for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
    {
        if(pKF==*lit)
        {
            lKFs.erase(lit);
            break;
        }
    }
    

}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());

    //zaki_change
    mvInvertedFile_vanilla.clear();
    mvInvertedFile_vanilla.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore, bool vanilla_flag)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;

    performancetools::Timer timer;
    timer.begin();
    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe

 

    {
       
        unique_lock<mutex> lock(mMutex);

       

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            if (!vanilla_flag)
            {
                list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFi=*lit;
                    if(pKFi->mnLoopQuery!=pKF->mnId)
                    {
                        pKFi->mnLoopWords=0;
                        if(!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->mnLoopQuery=pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                }
            }
            else
            {
                list<KeyFrame*> &lKFs =   mvInvertedFile_vanilla[vit->first];

                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFi=*lit;
                    if(pKFi->mnLoopQuery!=pKF->mnId)
                    {
                        pKFi->mnLoopWords=0;
                        if(!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->mnLoopQuery=pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                }
            }
        }
    }

    if(lKFsSharingWords.empty()){
        std::cout << "PERFORMANCE_DetectLoopCandidates " << timer.elapsed() << " id " << pKF->mnId << " candid size " << lKFsSharingWords.size() << std::endl;

        return vector<KeyFrame*>();
    }

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    std::cout << "loop candidates information " << std::endl;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        std::cout << pKFi->mnId << " " << pKFi->total_points << " " << pKFi->removed_so_far << " " << pKFi->removed_words << " " << pKFi->mnLoopWords << " " << minCommonWords << " ";

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            //zaki_change
            float si;
            if (!vanilla_flag)
                si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);
            else
                si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec_vanilla);

            std::cout << si;
            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
        else
        {
            pKFi->mLoopScore = 0.0;
        }
        std::cout << std::endl;
    }

    if(lScoreAndMatch.empty()){
        std::cout << "PERFORMANCE_DetectLoopCandidates " << timer.elapsed() << " id " << pKF->mnId << " candid size " << lKFsSharingWords.size() << std::endl;

        return vector<KeyFrame*>();
    }

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        std::cout << "checking " << pKFi->mnId << " score after accumuation " << accScore << " selected keyframe " << pBestKF->mnId << std::endl; 
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    std::cout << "min score " << minScoreToRetain << std::endl;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

std::cout << "PERFORMANCE_DetectLoopCandidates " << timer.elapsed() << " id " << pKF->mnId << " candid size " << lKFsSharingWords.size() << std::endl;


    return vpLoopCandidates;
}

vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, bool vanilla_flag)
{
    performancetools::Timer timer;
    timer.begin();



    list<KeyFrame*> lKFsSharingWords;

    

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            if (!vanilla_flag)
            {
                list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFi=*lit;

                    //zaki_temp_change
                //    if (pKFi->mnFrameId < 229)
                //    {

                        if(pKFi->mnRelocQuery!=F->mnId)
                        {
                            pKFi->mnRelocWords=0;
                            pKFi->mnRelocQuery=F->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                        pKFi->mnRelocWords++;
                //    }
                }
            }
            else
            {
                list<KeyFrame*> &lKFs =   mvInvertedFile_vanilla[vit->first];

                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFi=*lit;
                    if(pKFi->mnRelocQuery!=F->mnId)
                    {
                        pKFi->mnRelocWords=0;
                        pKFi->mnRelocQuery=F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }   
            }
        }
    }
    if(lKFsSharingWords.empty()){
        std::cout << "PERFORMANCE_DetectRelocalizationCandidates " << timer.elapsed() << " id " << F->mnId <<  std::endl;

        //zaki_change because we are calling the function twice after each other, we should revert back the ids
        //for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
        //{
        //    KeyFrame* pKFi = *lit;
        //    pKFi->mnRelocQuery = 0;
       // }


        return vector<KeyFrame*>();
    }

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        std::cout << pKFi->mnId << " " << pKFi->total_points << " " << pKFi->removed_so_far << " " << pKFi->removed_words << " " << pKFi->mnRelocWords << " " << minCommonWords << " ";
        if(pKFi->mnRelocWords>minCommonWords)
        {
            //zaki_change
           // if (!vanilla_flag)
           //     pKFi->draw(F->mnId);
            nscores++;
            
            //zaki_change
            float si;
            if (!vanilla_flag)
                si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            else
                si = mpVoc->score(F->mBowVec,pKFi->mBowVec_vanilla);

            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
            std::cout << si ;
        }
        else
        {
            pKFi->mRelocScore=0.0;
        }
        std::cout << std::endl;
    }

    if(lScoreAndMatch.empty()){
        std::cout << "PERFORMANCE_DetectRelocalizationCandidates " << timer.elapsed() << " id " << F->mnId <<  std::endl;
        
        //zaki_change
       // for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
       // {
       //     KeyFrame* pKFi = *lit;
       //     pKFi->mnRelocQuery = 0;
       // }

        return vector<KeyFrame*>();
    }

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }

        std::cout << "checking " << pKFi->mnId << " score after accumuation " << accScore << " selected keyframe " << pBestKF->mnId << std::endl; 

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    std::cout << "min score to retain " << minScoreToRetain << std::endl;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
    std::cout << "PERFORMANCE_DetectRelocalizationCandidates " << timer.elapsed() << " id " << F->mnId <<  std::endl;

    //zaki_change
    //for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    //{
    //    KeyFrame* pKFi = *lit;
    //    pKFi->mnRelocQuery = 0;
   // }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
