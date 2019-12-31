#include "map.hpp"

void MyMap::insertKeyFrame ( Frame::Ptr frame, MatchedPointMap& matchedpoints )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        // initialize the static weight use Uniform Distribution
        for(auto iter = frame->ForegroundEdge_.begin(); iter != frame->ForegroundEdge_.end(); iter++)
        {
            frame->ForegroundEdge_Ws[iter->first] = (1/(frame->ForegroundEdge_.size()));
        }

        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        for(auto iter = matchedpoints.begin(); iter != matchedpoints.end(); iter++)
        {
            frame->ForegroundEdge_Ws[iter->second] = keyframes_[frame->id_ - 3]->ForegroundEdge_Ws[iter->first];
        }
        keyframes_[ frame->id_ ] = frame;
    }
}

void MyMap::insertKeyFrame(Frame::Ptr frame)
{
    cout << "this is the first keyframe" << endl;
    // initialize the static weight use Uniform Distribution
    int count = 0;
    for(auto iter = frame->ForegroundEdge_.begin(); iter != frame->ForegroundEdge_.end(); iter++)
    {
        frame->ForegroundEdge_Ws[iter->first] = (1/(frame->ForegroundEdge_.size()));
        frame->ForegroundEdge_Ws_init[iter->first] = (1/(frame->ForegroundEdge_.size()));
        count++;
    }
    keyframes_.insert( make_pair(frame->id_, frame) );
 //   keyframes_[frame->id_] = frame;
    cout << "insert success" << endl;
}
