/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "camera.h"
#include "common.h"
    
// forward declare 
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    Eigen::Affine3f T_c_w_eigen;      // transform from world to camera
    SE3             T_c_w_;
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_,intensity_; // color and depth image
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    unordered_map<cv::Point2i,PointT,HashFunc,EqualKey> ForegroundEdge_;  // keyframe's foregroundEdge
    unordered_map<cv::Point2i,float,HashFunc,EqualKey> ForegroundEdge_Ws_init; // static weight points
    vector<cv::Point2i> ForegroundEdgePix;
    StaticWeightMap ForegroundEdge_Ws;
    bool                           is_key_frame_;  // whether a key-frame
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, Eigen::Affine3f T_c_w_e = Eigen::Affine3f::Identity(), SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth(const cv::Point2i &PixelCoor );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose(const Eigen::Affine3f T_c_w_estimate );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
    // rgb to gray 0.299*R + 0.587*G + 0.114 * B
    void RGB2GRAY();

    // get pixel's 3D pointcloud coordinate
    Vector3d getPointCloudCoor(const cv::Point2i& PixelCoor);
    // get pixel's Intensity
    double getPixelIntensity(const cv::Point2i& PixelCoor);

    // calculate the static weight of foreground edge points
    void UpdateStaticWeight(vector<float> &geoResiduals, vector<float> &geoResiduals_all, MatchedPointMap &matchedpoints, unsigned long currentFrame_id);


};


#endif // FRAME_H
