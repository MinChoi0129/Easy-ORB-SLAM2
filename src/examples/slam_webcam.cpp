#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// Linux에서 비동기 키 입력 체크를 위한 간단한 함수
bool kbhit() {
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return true;
  }

  return false;
}

bool GetAsyncKeyState(int key) {
  if (kbhit()) {
    int ch = getchar();
    return (ch == key || ch == (key - 32)); // 대소문자 모두 체크
  }
  return false;
}

#include <System.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>

using namespace std;

// 사용자로부터 맵 파일 이름을 입력받는 함수 : 저장되는 맵 파일 이름에 이용됨
inline void UserInputMapFileName(string &mapFileName);

// default camera가 정상적으로 open되었는지, 맵 디렉토리가 존재하는지 확인하는
// 함수. error: true 반환
bool ErrorCheck(const cv::VideoCapture &capture);

int main(void) {
  string userInputMapFileName;
  UserInputMapFileName(userInputMapFileName);

  cv::VideoCapture capture(0); // default camera open
  if (ErrorCheck(capture))
    return 1;

  capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  // slam_video -> slam -> save
  // slam_video : 프로그램 실행시 항상 새로운 맵을 생성 (mapSave, reuseMap
  // 매개변수와 연관)
  const bool mapSave = true, reuseMap = false;
  /* parameter form
  ** System(vocabulary, settingFile, sensor, useViewer = true, mapSave = false,
  * reuseMap = false, setMapFileName = "", loadMapFileName = ""); */

  const string VOCABULARY_PATH = "../data/vocabulary/";
  const string FILE_NAME = "ORBvoc.bin";
  ifstream txtVocaFileCheck; // file open 확인 용도
  txtVocaFileCheck.open(VOCABULARY_PATH + FILE_NAME);
  if (txtVocaFileCheck.eof() || !txtVocaFileCheck.is_open()) {
    cerr << "\n" << VOCABULARY_PATH + FILE_NAME << " : File not found.\n";
    cout << "Press any key to exit..." << endl;
    return 0;
  }
  txtVocaFileCheck.close();

  // vocabulary type : txt
  // ORB_SLAM2::System SLAM("../data/vocabulary/ORBvoc.txt",
  // "../data/parameter/Monocular/TUM2.yaml", ORB_SLAM2::System::MONOCULAR,
  // true, mapSave, reuseMap, userInputMapFileName);

  // vocabulary type : bin (binary)
  ORB_SLAM2::System SLAM(VOCABULARY_PATH + FILE_NAME,
                         "../data/parameter/Monocular/TUM2.yaml",
                         ORB_SLAM2::System::MONOCULAR, true, mapSave, reuseMap,
                         userInputMapFileName);

  cout << endl << "-------" << endl;
  cout << "Start processing video sequences ..." << endl;
  cout << "Key-stroke \'s\' : End program loop." << endl;

  cv::Mat video;
  vector<double> vTimestamps;
  chrono::system_clock::time_point startTime, endTime;
  startTime = chrono::system_clock::now();
  while (true) {
    capture >> video;
    /// double timestamp = capture.get(cv::CAP_PROP_POS_MSEC) / 1000.; //
    /// cv::CAP_PROP_FPS only works on videos.  Using a webcam, the return value
    /// is -1
    endTime = chrono::system_clock::now();
    double timestamp = chrono::duration<double>(endTime - startTime).count();
    cv::Mat Tcw = SLAM.TrackMonocular(video, timestamp);
    std::cout << Tcw << std::endl;
    vTimestamps.push_back(timestamp);
    if (GetAsyncKeyState('s') || GetAsyncKeyState('S'))
      break; // 's' Key 입력시 반복문을 빠져나온다.
  }
  SLAM.Shutdown();
  capture.release();

  sort(vTimestamps.begin(), vTimestamps.end());

  double totalTime = 0.;
  size_t nSize = vTimestamps.size();
  for (double elem : vTimestamps)
    totalTime += elem;
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimestamps[nSize / 2] << endl;
  cout << "mean tracking time: " << totalTime / nSize << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void UserInputMapFileName(string &mapFileName) {
  cout << "Enter the name of the map file to be saved : "; // 확장자를 포함하지
                                                           // 않아도 된다.
  // 저장되는 맵 파일 이름은 map - " + videoFile(사용자 입력) + ".bin" 형태로
  // 구성된다.
  getline(cin, mapFileName);
}

bool ErrorCheck(const cv::VideoCapture &capture) {
  bool isError = false;

  // default camera가 정상적으로 open되었는지를 확인
  if (!capture.isOpened()) {
    isError = true;
    cerr << "default webcam open error." << endl;
  }

  // 맵 파일을 저장하는[map]디렉토리가 존재하는지 확인하는 함수
  if (![]() {
        boost::filesystem::path mapDir("../data/map/");
        if (boost::filesystem::create_directory(
                mapDir)) // map 디렉토리를 생성한다. 디렉토리를 생성한 경우 true
                         // 반환, 생성하지 않은 경우 false 반환
          cout << "Map directory not found. Create a directory."
               << " (ORB-SLAM/data/map/)" << endl;
        else
          cout << "Map directory checking OK." << " (ORB-SLAM/data/map/)"
               << endl;
        return boost::filesystem::exists(mapDir)
                   ? true
                   : false; // 디렉토리가 존재하는지 한번 더 확인한다
      }()) {
    // 디렉토리 생성을 시도했으나 디렉토리가 생성되지 않은 경우
    isError = true;
    cerr << "Directory not found. No directory created." << endl;
  }
  if (isError)
    return isError;
  return false;
}
