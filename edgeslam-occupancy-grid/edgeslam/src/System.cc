#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{
System::System(const string &strVocFile, const string &strSettingsFile, std::string rt, const eSensor sensor, const bool bUseViewer):
    mSensor(sensor), RunType(std::move(rt)), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false)
{
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    mpMap = new Map();

    if (RunType.compare("client") == 0){
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

        new thread(&ORB_SLAM2::System::SaveMapPointsLoop, this);
        new thread(&ORB_SLAM2::System::SaveAllPosesLoop, this);
        new thread(&ORB_SLAM2::System::SaveNewestPoseLoop, this);
    } else if (RunType.compare("server") == 0){
        mpLocalMapper = new LocalMapping(mpMap, mpKeyFrameDatabase, mpVocabulary, strSettingsFile, mSensor==MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    }

    if (RunType.compare("client") == 0){
        if(bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }
    }

    if (RunType.compare("server") == 0){
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            cout << "log,System::TrackStereo(),localization mode branch" << std::endl;

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            if (RunType.compare("client") == 0){
                mpTracker->Reset();
            } else if (RunType.compare("server") == 0){
                mpLocalMapper->RequestReset();
            }

            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            cout << "log,System::TrackRGBD(),localization mode branch" << std::endl;

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            if (RunType.compare("client") == 0){
                mpTracker->Reset();
            } else if (RunType.compare("server") == 0){
                mpLocalMapper->RequestReset();
            }

            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            cout << "log,System::TrackMonocular(),localization mode branch" << std::endl;

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            if (RunType.compare("client") == 0){
                mpTracker->Reset();
            } else if (RunType.compare("server") == 0){
                mpLocalMapper->RequestReset();
            }

            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ClientShutdown()
{
    mpTracker->~Tracking();

    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("Edge-SLAM: Map Viewer");

    usleep(5000);
}

void System::ServerShutdown()
{
    mpLocalMapper->RequestFinish();

    mpLoopCloser->RequestFinish();

    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    usleep(5000);
}

void System::SaveMapPointsLoop() {
    while (1) {
        ofstream f;
        f.open(mapPointsFilename.c_str());

        if (!f.is_open()) {
            std::cerr << "ERROR: Failed to open map points file: " << mapPointsFilename << std::endl;
            sleep(1);
            continue;
        }

        mpMap->mMutexMapUpdate.lock();
        vector<MapPoint*> mapPoints = mpMap->GetAllMapPoints();
        vector<cv::Mat> worldPos;
        for (size_t i=0; i<mapPoints.size(); i++) {
            worldPos.push_back(mapPoints[i]->GetWorldPos());
        }
        mpMap->mMutexMapUpdate.unlock();

        for (size_t i=0; i<worldPos.size(); i++) {
            f << worldPos[i].at<float>(0,0) << " " << worldPos[i].at<float>(0,1) << " " << worldPos[i].at<float>(0,2) << endl;
        }

        f.close();
        sleep(1);
    }

}


void System::SaveNewestPoseLoop() {
    while (1) {
        ofstream f;
        f.open(newestPoseFilename.c_str());

        if (!f.is_open()) {
            std::cerr << "ERROR: Failed to open newest pose file: " << newestPoseFilename << std::endl;
            sleep(1);
            continue;
        }

        mpMap->mMutexMapUpdate.lock();
        int newestKeyFrameId = KeyFrame::nNextId;
        KeyFrame* keyFrame = mpMap->RetrieveKeyFrame(newestKeyFrameId - 1);
        if (keyFrame == nullptr) {
            std::cout << "No key frames exist yet" << std::endl;
            mpMap->mMutexMapUpdate.unlock();
            sleep(1);
            continue;
        }
        cv::Mat poseMatrix = keyFrame->GetPose();
        mpMap->mMutexMapUpdate.unlock();

        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                f << poseMatrix.at<float>(r, c);
                if (c < poseMatrix.cols - 1) {
                    f << " ";
                }
            }
            f << "\n";
        }

        f.close();
        sleep(1);
    }
}

void System::SaveAllPosesLoop() {
    while (1) {
        ofstream f;
        f.open(allPosesFilename.c_str());

        if (!f.is_open()) {
            std::cerr << "ERROR: Failed to open all poses file: " << allPosesFilename << std::endl;
            sleep(1);
            continue;
        }

        mpMap->mMutexMapUpdate.lock();
        vector<KeyFrame*> keyFrames = mpMap->GetAllKeyFrames();
        for (int i = 0; i < keyFrames.size(); i++) {
            cv::Mat poseMatrix = keyFrames[i]->GetPose();
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    f << poseMatrix.at<float>(r, c);
                    if (c < poseMatrix.cols - 1) {
                        f << " ";
                    }
                }
                f << "\n";
            }
            f << "\n";
        }
        mpMap->mMutexMapUpdate.unlock();

        f.close();
        sleep(1);
    }
}

void System::SaveMapPoints(const string &filename)
{
  cout << endl << "Saving map points to " << filename << " ..." << endl;
  vector<MapPoint*> mapPoints = mpMap -> GetAllMapPoints();
  
  vector<cv::Mat> worldPos;
  for (size_t i=0; i<mapPoints.size(); i++) {
    worldPos.push_back(mapPoints[i]->GetWorldPos());
  }

  ofstream f;
  f.open(filename.c_str());

  for (size_t i=0; i<worldPos.size(); i++) {
    f << worldPos[i].at<float>(0,0) << " " << worldPos[i].at<float>(0,1) << " " << worldPos[i].at<float>(0,2) << endl;
  }

  f.close();
  cout << "Map points saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

}
