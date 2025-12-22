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

// 사용자로부터 맵 파일 이름을 입력받는 함수, 불러오는 맵 파일을 입력받아야하며
// 확장자를 포함시켜야 한다.
inline void UserInputLoadMapFileName(string &mapFileName);

// default camera가 정상적으로 open되었는지, 맵 디렉토리가 존재하며 맵 파일이
// 존재하는지 확인하는 함수, error: true 반환
bool ErrorCheck(const cv::VideoCapture &capture, const string &mapFileName);

int main(void) {
  string userInputMapFileName;
  UserInputLoadMapFileName(userInputMapFileName);
  cv::VideoCapture capture(0); // default camera open

  if (ErrorCheck(capture, userInputMapFileName))
    return 1;

  capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  // relocalization_video -> load -> slam
  // 프로그램 실행시 입력받은 비디오 파일 이름과 맵 파일 이름을 입력받음
  // 맵 파일을 로드하고 이어서 slam을 수행하되 맵 파일에 저장시키지
  // 않음(mapSave, reuseMap 매개변수와 연관)
  const bool mapSave = false, reuseMap = true;
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
  // true, mapSave, reuseMap, userInputMapFileName, userInputMapFileName);

  // vocabulary type : bin (binary)
  ORB_SLAM2::System SLAM(VOCABULARY_PATH + FILE_NAME,
                         "../data/parameter/Monocular/TUM2.yaml",
                         ORB_SLAM2::System::MONOCULAR, true, mapSave, reuseMap,
                         userInputMapFileName, userInputMapFileName);

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
    /// cv::CAP_PROP_FPS only works on videos. Using a webcam, the return value
    /// is -1
    endTime = chrono::system_clock::now();
    double timestamp = chrono::duration<double>(endTime - startTime).count();
    SLAM.TrackMonocular(video, timestamp);
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

void UserInputLoadMapFileName(string &mapFileName) {
  cout << "Map file path : ORB-SLAM/data/map\n";
  cout << "Enter the name of the map file to load. (Include file extensions : "
          "*.bin) : ";
  // path : ORB-SLAM/data/map
  getline(cin, mapFileName);
}

bool ErrorCheck(const cv::VideoCapture &capture, const string &mapFileName) {
  bool isError = false;

  // default camera가 정상적으로 open되었는지를 확인
  if (!capture.isOpened()) {
    isError = true;
    cerr << "default webcam open error." << endl;
  }

  // 맵 파일을 저장하는 [map]디렉토리가 존재하는지 확인
  if (![&]() {
        boost::filesystem::path mapDir("../data/map/");
        return boost::filesystem::exists(mapDir) ? true : false;
      }()) {
    isError = true;
    cerr << "Map data directory path does not exist." << endl;
  }

  // [map]디렉토리에 string매개변수와 일치하는 맵 파일이 존재하는지 확인
  if (![&]() {
        boost::filesystem::path mapFile("../data/map/" + mapFileName);
        return boost::filesystem::exists(mapFile) ? true : false;
      }()) {
    isError = true;
    cerr << "map data file does not exist (File name : " + mapFileName + ")"
         << endl;
  }

  if (isError)
    return isError;
  return false;
}
