

#include "System.h"
#include "Converter.h"
#include <iomanip>
#include <pangolin/pangolin.h>
#include <thread>
#include <unistd.h>

bool has_suffix(const string &str, const string &suffix) {
  if (str.empty() || suffix.empty())
    return false;
  size_t index = str.find(
      suffix,
      str.length() - suffix.length()); // str.length() - suffix.length() < 0 :
                                       // ?????? ????? error?? ???????? ??��?.
  return index != string::npos;
}

string GetNonExtVocabularyFileName(string vocaFileName) {
  if (vocaFileName.find_last_of('/') != string::npos)
    vocaFileName.erase(vocaFileName.begin(),
                       vocaFileName.begin() + vocaFileName.find_last_of('/') +
                           1);

  if (vocaFileName.find('.') != string::npos) {
    reverse(vocaFileName.begin(), vocaFileName.end());
    vocaFileName.erase(vocaFileName.begin(),
                       vocaFileName.begin() + vocaFileName.find('.') + 1);
    reverse(vocaFileName.begin(), vocaFileName.end());
  }

  return vocaFileName;
}

namespace ORB_SLAM2 {
/*System::System(const string &strVocFile, const string &strSettingsFile, const
   eSensor sensor, const bool bUseViewer, const string videoFileName, bool
   is_save_map_, bool reuseMap) :mSensor(sensor), is_save_map(is_save_map_),
        mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),
   mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)*/

System::System(const string &strVocFile, const string &strSettingsFile,
               const eSensor sensor, const bool bUseViewer, bool is_save_map_,
               bool reuseMap, const string setMapFileName,
               const string loadMapFileName)
    : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(is_save_map_), is_save_map(is_save_map_)

{
  // Output welcome message
  cout << endl
       << "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of "
          "Zaragoza."
       << endl
       << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
       << "This is free software, and you are welcome to redistribute it"
       << endl
       << "under certain conditions. See LICENSE.txt." << endl
       << endl;

  cout << "Input sensor was set to: ";

  if (mSensor == MONOCULAR)
    cout << "Monocular" << endl;
  else if (mSensor == STEREO)
    cout << "Stereo" << endl;
  else if (mSensor == RGBD)
    cout << "RGB-D" << endl;

  // Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  if (!setMapFileName.empty()) {
    if (!reuseMap) // slam_video
      mapFile = "map-" + setMapFileName +
                ".bin"; // slam_video ????? setMapFileName?? map???? ???????
                        // ??????? ??????. (default map file name)
    else if (!loadMapFileName.empty()) // relocalization_video
      mapFile = loadMapFileName; // relocalization_video ????? ????? map??????
                                 // ????? ?? ???????
                                 // relocalization_video.cpp???? ??��??? ??????
                                 // ????? map load?? ?????? ???? ??????.
  }

  // Load ORB Vocabulary
  cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

  mpVocabulary = new ORBVocabulary();
  bool bVocLoad = false;
  if (has_suffix(strVocFile, ".bin")) {
    bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
  } else if (has_suffix(strVocFile,
                        ".txt")) { // vocabulary file name extension : .txt
    bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (bVocLoad) { // if '.txt vocabulary file' loaded success.
      cout << "Vocabulary loaded success.\nTrying to create a binary "
              "vocabulary file to speed up the program."
           << endl;
      const string vocaFileName = GetNonExtVocabularyFileName(strVocFile);
      const string vocabularyBinaryFileName =
          "../data/vocabulary/" + vocaFileName +
          ".bin"; // (save path) + (txt vocabulary file name) + (.bin)
      mpVocabulary->saveToBinaryFile(vocabularyBinaryFileName);
      cout << "Binary vocabulary file created successfully." << endl;
      cout << "Vocabulary file : " << vocabularyBinaryFileName << endl;
    }
  } else {
    bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
  }

  if (!bVocLoad) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << strVocFile << endl;
    exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;

  // Create KeyFrame Database
  // Create the Map
  if (reuseMap && !mapFile.empty()) {
    LoadMap(mapFile);
  } else {
    mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
    mpMap = new Map();
  }

  // Create Drawers. These are used by the Viewer
  mpFrameDrawer = new FrameDrawer(mpMap, reuseMap);
  mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this
  // constructor)
  mpTracker =
      new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap,
                   mpKeyFrameDatabase, strSettingsFile, mSensor, reuseMap);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
  mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
                                 mSensor != MONOCULAR);
  mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

  // Initialize the Viewer thread and launch
  if (bUseViewer) {
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                          strSettingsFile, reuseMap);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
  }

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                            const double &timestamp) {
  if (mSensor != STEREO) {
    cerr << "ERROR: you called TrackStereo but input sensor was not set to "
            "STEREO."
         << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                          const double &timestamp) {
  if (mSensor != RGBD) {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD."
         << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
  if (mSensor != MONOCULAR) {
    cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
            "Monocular."
         << endl;
    exit(-1);
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      mpTracker->InformOnlyTracking(true); // localization mode
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false); // slam mode
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

void System::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpMap->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

void System::Reset() {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
}

void System::Shutdown() {
  cout << "\n============ Shutdown ============\n" << endl;
  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();
  if (mpViewer) {
    mpViewer->RequestFinish();
    while (!mpViewer->isFinished()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
  }

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
         mpLoopCloser->isRunningGBA()) {
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
  if (mpViewer)
    pangolin::BindToContext("ORB-SLAM2: Map Viewer");

  if (is_save_map) // is_save_map type : bool; ?? ???? ???��? ???? SaveMap(type
                   // : string)?? ??????.
    SaveMap(mapFile); // map save
  cout << "is_save_map value : " << (is_save_map ? "true" : "false");
  cout << "\n==================================" << endl;
}

void System::SaveTrajectoryTUM(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                               lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL)
      continue;

    KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    vector<float> q = Converter::toQuaternion(Rwc);

    f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0)
      << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0]
      << " " << q[1] << " " << q[2] << " " << q[3] << endl;
  }
  f.close();
  cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  cout << endl
       << "Saving keyframe trajectory to " << filename << " ..." << endl;

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad())
      continue;

    cv::Mat R = pKF->GetRotation().t();
    vector<float> q = Converter::toQuaternion(R);
    cv::Mat t = pKF->GetCameraCenter();
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
      << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
      << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
  }

  f.close();
  cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                               lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    ORB_SLAM2::KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    while (pKF->isBad()) {
      //  cout << "bad parent" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1)
      << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " "
      << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " "
      << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " "
      << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " "
      << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
  }
  f.close();
  cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

void System::SaveMap(const string &filename) {
  cout << "====System::SaveMap(...) Called====" << endl;
  ofstream out("../data/map/" + filename, std::ios_base::binary);
  if (!out) {
    cerr << "Cannot Write to Mapfile: " << mapFile << endl;
    // cin.get();
    exit(-1);
  }
  cout << "Saving Mapfile: " << mapFile
       << " (path : ORB-SLAM/data/map/" + filename + ")" << std::flush;
  boost::archive::binary_oarchive oa(out, boost::archive::no_header);
  oa << mpMap;
  oa << mpKeyFrameDatabase;
  cout << " ...done" << std::endl;
  out.close();
  cout << "===================================" << endl;
}
bool System::LoadMap(const string &filename) {
  cout << "====System::LoadMap(...) Called====" << endl;
  ifstream in("../data/map/" + filename, std::ios_base::binary);
  if (!in) {
    cerr << "Cannot Open Mapfile: " << mapFile << " , Create a new one" << endl;
    return false;
  }
  cout << "Loading Mapfile: " << mapFile
       << " (path : ORB-SLAM/data/map/" + filename + ")" << std::flush;
  boost::archive::binary_iarchive ia(in, boost::archive::no_header);
  ia >> mpMap;
  ia >> mpKeyFrameDatabase;
  mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
  cout << " ...done" << endl;
  cout << "Map Reconstructing" << flush;
  vector<ORB_SLAM2::KeyFrame *> vpKFS = mpMap->GetAllKeyFrames();
  unsigned long mnFrameId = 0;
  for (auto it : vpKFS) {
    it->SetORBvocabulary(mpVocabulary);
    it->ComputeBoW();
    if (it->mnFrameId > mnFrameId)
      mnFrameId = it->mnFrameId;
  }
  Frame::nNextId = mnFrameId;
  cout << " ...done" << endl;
  in.close();
  cout << "===================================" << endl;
  return true;
}

} // namespace ORB_SLAM2
