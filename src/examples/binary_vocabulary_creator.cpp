#define _CRT_SECURE_NO_DEPRECATE
#pragma warning(disable : 4996)

#include "ORBVocabulary.h"
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

string GetNonExtVocabularyFileName(string vocaFileName) {
  if (string::npos != vocaFileName.find('.'))
    vocaFileName.erase(vocaFileName.begin() + vocaFileName.find_last_of('.'),
                       vocaFileName.end());
  return vocaFileName;
}

string GetFileName(string argv) {
  argv.erase(argv.begin(), argv.begin() + argv.find_last_of('\\') + 1);
  return argv;
}

string GetFilePath(string argv) {
  argv.erase(argv.begin() + argv.find_last_of('\\') + 1, argv.end());
  return argv;
}

int main(int argc, const char **argv) {
  string vocaFileName;
  const string &VOCABULARY_PATH = "../data/vocabulary/";
  ORB_SLAM2::ORBVocabulary mpVocabulary;
  fstream vocaFile;
  cout << "Converts a text vocabulary file to a binary vocabulary file.\n";

  if (argc == 1) {
    cout << "The file name must include an extension.\n";
    cout << "Input file name : ";
    getline(cin, vocaFileName);
  } else if (argc == 2) {
    vocaFileName = GetFileName(argv[1]);
    const_cast<string &>(VOCABULARY_PATH) = GetFilePath(argv[1]);
    cout << "File name : " << vocaFileName << endl;
  }

  vocaFile.open(VOCABULARY_PATH + vocaFileName);
  if (vocaFile.eof() || !vocaFile.is_open()) {
    cerr << "File not found." << endl;
    cerr << "Path : " << VOCABULARY_PATH + vocaFileName << '\n' << endl;
  } else { // File Found.
    vocaFile.close();
    cout << "File path : " << VOCABULARY_PATH + vocaFileName << endl;
    cout << "Loading file ... ";
    bool textFileValid =
        mpVocabulary.loadFromTextFile(VOCABULARY_PATH + vocaFileName);
    if (textFileValid) { // text vocabulary file valid.
      cout << "File loading completed.\n";
      const string BINARY_VOCABULARY_FILE_NAME =
          GetNonExtVocabularyFileName(vocaFileName) + ".bin";
      mpVocabulary.saveToBinaryFile(VOCABULARY_PATH +
                                    BINARY_VOCABULARY_FILE_NAME);
      cout << "File conversion complete." << endl;
      cout << "Binary vocabulary file : " << BINARY_VOCABULARY_FILE_NAME
           << endl;
      cout << "Binary vocabulary file path : "
           << VOCABULARY_PATH + BINARY_VOCABULARY_FILE_NAME << '\n'
           << endl;
    } else { // text vocabulary file invalid.
      cerr << "Vocabulary file loading fail.\n" << endl;
    }
  }
  cout << "Press any key to exit..." << endl;
  return 0;
}