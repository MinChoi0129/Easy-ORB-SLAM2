

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <list>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

#include "BoostArchiver.h"
#include <mutex>

namespace ORB_SLAM2 {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
public:
  KeyFrameDatabase(ORBVocabulary *voc);

  void add(KeyFrame *pKF);

  void erase(KeyFrame *pKF);

  void clear();

  // Loop Detection
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

  // Relocalization
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

public:
  // for serialization
  KeyFrameDatabase() {}
  void SetORBvocabulary(ORBVocabulary *porbv) { mpVoc = porbv; }

private:
  // serialize is recommended to be private
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);

protected:
  // Associated vocabulary
  ORBVocabulary *mpVoc;

  // Inverted file
  std::vector<list<KeyFrame *>> mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} // namespace ORB_SLAM2

#endif
