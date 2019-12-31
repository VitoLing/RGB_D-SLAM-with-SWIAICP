#ifndef CAMERA_TRACKER_H_
#define CAMERA_TRACKER_H_



#include "common.h"
#include "iaicp.h"
#include "map.hpp"

using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;
using namespace std;
using namespace cv;


typedef Matrix<float,6,1> Vector6f;
class CameraTracker
{
public:


    CameraTracker();
    ~CameraTracker();

    void run(CloudPtr scene);   //feed in the currrent received Cloud
    void run(Frame::Ptr pFrame);   //feed in rgb and depth image
    void recalibrate(CloudPtr &cloud);

    CloudPtr currentFrame;
    Eigen::Affine3f m_key2cur;
    Eigen::Affine3f m_pose, m_lastpose;
    Eigen::Affine3f m_keypose;

   // CloudPtr getRoi();
    Eigen::Affine3f getPose();
    void getXYZQ(float &tx, float &ty, float &tz, float &qx, float &qy, float &qz, float &qw);
    CloudPtr getKeyframes();
    CloudPtr getTransformedScene();  //get transformed current frame

    Vector6f m_speed, m_lastspeed, m_acc;
    CloudPtr m_map;
    MyMap::Ptr MyMapPtr;

    Point2i warp(PointT pt);
    PointT unwarp(Point2i coord, float depth);
    CloudPtr Mat2Cloud(Mat &imR, Mat &imD);
    vector<float> geoResdiuals_all,geoResdiuals;
    MatchedPointMap matchedPoints;
    void sampleForegroundEdge(CloudPtr &m_src, Frame::Ptr pFrame); //sample salient points in the source frame
    void checkAngles(Vector6f &vec);
    Eigen::Affine3f toEigen(Vector6f pose);
    Vector6f toVector(Eigen::Affine3f pose);
    float max_4(float diff1,float diff2, float diff3, float diff4);




private:
    Iaicp icpEst;
    bool m_changeKey;
    double m_lasttime, m_thistime;
    CloudPtr m_current, m_keyframe, m_transformedModel;
    vector<cv::Point2i> m_ForegroundEdgePixel;
    unordered_map<cv::Point2i,PointT,HashFunc,EqualKey> m_ForegroundEdge; // foreground of pointcloud
    Frame::Ptr currentKeyFrame,lastFrame;
    int m_cnt;
    float fx, fy, cx, cy; //camera parameters    
    int width, height;
};



#endif // CAMERA_TRACKER_H
