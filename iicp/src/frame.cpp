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

#include "frame.h"

Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp,Eigen::Affine3f T_c_w_e, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_eigen(T_c_w_e), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

double Frame::findDepth ( const cv::Point2i& PixelCoor )
{
    int x = PixelCoor.x;
    int y = PixelCoor.y;
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

void Frame::setPose (const Affine3f T_c_w_estimate )
{
    T_c_w_eigen = T_c_w_estimate;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

void Frame::RGB2GRAY()
{
   cv::cvtColor(color_,intensity_,cv::COLOR_RGB2GRAY);
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

Vector3d Frame::getPointCloudCoor(const cv::Point2i &PixelCoor)
{
    Vector2d temp(PixelCoor.x,PixelCoor.y);
    return camera_->pixel2camera(temp, findDepth(PixelCoor));
}

double Frame::getPixelIntensity(const cv::Point2i &PixelCoor)
{
    return intensity_.at<double>(PixelCoor.y,PixelCoor.x);
}

void Frame::UpdateStaticWeight(vector<float> &geoResiduals,vector<float> &geoResiduals_all,MatchedPointMap &matchedpoints, unsigned long currentFrame_id)
{
    sort(geoResiduals.begin(),geoResiduals.end());
    float middle = 1.4826 * geoResiduals[geoResiduals.size() - geoResiduals.size()/2];
    for(int i = 0; i < ForegroundEdgePix.size(); ++i)
    {
        auto got = matchedpoints.find(ForegroundEdgePix[i]);
        if(got != matchedpoints.end())
        {
            ForegroundEdge_Ws[ForegroundEdgePix[i]] = 0.5 * (7/(7+currentFrame_id - id_)) *  ForegroundEdge_Ws_init[ForegroundEdgePix[i]] + (1 - 0.5*(7/(7+currentFrame_id - id_))) * (11/(10 + pow((geoResiduals_all[i] - middle)/2,2)));
        }
        else {
            ForegroundEdge_Ws[ForegroundEdgePix[i]] = 11/(10 + pow((geoResiduals_all[i] - middle)/2,2));
        }
    }
}

