#ifndef MAP_HPP
#define MAP_HPP
#include "common_include.h"
#include "common.h"
#include "frame.h"

class MyMap
{
public:
    typedef shared_ptr<MyMap> Ptr;
    unordered_map<unsigned long, Frame::Ptr >  keyframes_;        // all landmarks
 //   list<Frame::Ptr> keyframes_;

    MyMap() {}

    void insertKeyFrame(Frame::Ptr frame, MatchedPointMap &matchedpoints);
    void insertKeyFrame(Frame::Ptr frame);
};

#endif // MAP_HPP
