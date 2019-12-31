#include "camera_tracker.h"
#include <Eigen/Geometry>
#include "config.h"
using namespace pcl;


CameraTracker::CameraTracker():MyMapPtr(new MyMap)
{
    fx = Config::get<float>("/fx");
    fy = Config::get<float>("/fy");
    cx = Config::get<float>("/cx");
    cy = Config::get<float>("/cy");
    height = Config::get<float>("/height");
    width = Config::get<float>("/width");

    m_cnt = 0;
    m_key2cur = toEigen(toVector(Eigen::Affine3f::Identity()));
    m_pose = toEigen(toVector(Eigen::Affine3f::Identity()));
    m_lastpose = toEigen(toVector(Eigen::Affine3f::Identity()));
    m_keypose = toEigen(toVector(Eigen::Affine3f::Identity()));
    /*
    m_key2cur = Eigen::Affine3f::Identity();
    m_pose = Eigen::Affine3f::Identity();
    m_lastpose = Eigen::Affine3f::Identity();
    m_keypose = Eigen::Affine3f::Identity();
    */
    m_speed=Vector6f::Zero();
    m_acc=Vector6f::Zero();

    m_map.reset(new Cloud());
    m_current.reset(new Cloud);
    m_thistime = m_lasttime = 0;
}


CameraTracker::~CameraTracker()
{

}

void CameraTracker::run(CloudPtr scene)
{
    cout<<"width is "<<scene->width<<" height is: "<<scene->height<<"timestamp: "<<scene->header.stamp<<endl;
//    recalibrate(scene);
    if (m_cnt==0) {  //for the first frame data
        m_current.reset(new Cloud());
        m_current = scene;
        m_keyframe.reset(new Cloud());
        m_keyframe = scene;
        icpEst.setupSource(m_ForegroundEdge,m_ForegroundEdgePixel);
    }else{
        Vector6f temp = toVector(m_key2cur);
        checkAngles(temp);
        //change keyframe if key2current is greater than certain threshold
        float dthr=0.07f, athr = (8.f/180)*M_PI;
        if((sqrtf(pow(temp(0),2)+ pow(temp(1),2)+ pow(temp(2),2))> dthr|| sqrtf(pow(temp(3),2) + pow(temp(4),2) + pow(temp(5),2))>athr)){
//        //change key frame very  nth frame
//        if(m_cnt%10==1){
            m_changeKey=false;
            m_keyframe.reset(new Cloud);
            m_keyframe = m_current;
            m_key2cur=Eigen::Affine3f::Identity();
            m_keypose = m_pose;
            icpEst.setupSource(m_ForegroundEdge,m_ForegroundEdgePixel);
        }
        m_current.reset(new Cloud);
        m_current = scene;        
    }

    //estimate speed/acc
    m_lastspeed = m_speed;
    double dtime = m_thistime-m_lasttime;
    if (dtime< 1e6f*0.02f) {dtime=1e6f *0.03f;}
    m_speed= toVector(m_pose * m_lastpose.inverse());
    checkAngles(m_speed); m_speed = (m_speed/dtime) /**0.5f + m_lastspeed*0.5f*/;
    m_acc = (m_speed-m_lastspeed);
    checkAngles(m_acc);  m_acc = (m_acc/dtime);
    m_lasttime = m_thistime;
    m_thistime = (double)scene->header.stamp;
    cout<<"time interval is "<<m_thistime-m_lasttime<<endl;

    m_cnt++;
    //predict key2cur pose
    dtime = m_thistime-m_lasttime;
    if (dtime > 1e6f*0.04f) {dtime=1e6f *0.03f;}
    if(m_cnt>12)
       m_key2cur = toEigen(toVector(toEigen((m_speed*dtime + 0.5f*m_acc*pow(dtime, 2)) * 0.7f) * m_key2cur));

    icpEst.setupTarget(m_ForegroundEdge,m_ForegroundEdgePixel);
    icpEst.setupPredict_(m_key2cur);
   // icpEst.run();
    m_key2cur =  toEigen(toVector(icpEst.getTransResult()));

    m_lastpose=m_pose;
    m_pose = toEigen(toVector(m_key2cur*m_keypose));


    cout<<"current camera pose realtive to first frame is: "<<endl<<m_pose.matrix()<<endl;

    //mapping : store frame every 20th frame
    if (m_cnt%20==1){
        CloudPtr temp(new Cloud());
        for(size_t i=0; i<width/8; i++){
            for(size_t j=0; j<height/8; j++){
                temp->points.push_back(m_current->points[j*8*width+i*8]);
            }
        }
        transformPointCloud(*temp, *temp, m_pose.inverse());
        *m_map += *temp;
    }
}


CloudPtr CameraTracker::getTransformedScene()
{
    m_transformedModel.reset(new Cloud);
    CloudPtr temp; temp.reset(new Cloud);
    for(size_t i=0; i<width/4; i++){
        for(size_t j=0; j<height/4; j++){
            temp->points.push_back(m_current->points[j*4*width+i*4]);
        }
    }
    transformPointCloud(*temp, *m_transformedModel, m_pose.inverse());
    return m_transformedModel;
}

/*
CloudPtr CameraTracker::getRoi()
{
    return icpEst.getForegroundSource();
}

Affine3f CameraTracker::getPose()
{
    return m_pose;
}
*/

void  CameraTracker::getXYZQ(float &tx, float &ty, float &tz, float &qx, float &qy, float &qz, float &qw)
{
    Eigen::Affine3f inv_ = m_pose.inverse();
    inv_ = toEigen(toVector(inv_));
    Vector6f pose = toVector(inv_);
    tx=pose(0); ty=pose(1); tz=pose(2);
    Quaternion<float> q(inv_.rotation());
    q.normalize();
    qx=q.x(); qy=q.y(); qz=q.z(); qw=q.w();
}


CloudPtr CameraTracker::getKeyframes()
{
    return m_map;
}

Point2i CameraTracker::warp(PointT pt)
{
    Point2i pt2i;
    pt2i.x = int(floor(fx/pt.z * pt.x + cx));
    pt2i.y = int(floor(fy/pt.z * pt.y + cy));
    return pt2i;
}

PointT CameraTracker::unwarp(Point2i coord, float depth)
{
    PointT pt3d;
    pt3d.z = depth;
    pt3d.x = (float(coord.x) - cx) * depth / fx;
    pt3d.y = (float(coord.y) - cy) * depth / fy;
    return pt3d;
}


void CameraTracker::recalibrate(CloudPtr &cloud)
{
    for(size_t i=0; i<width; i++){
        for(size_t j=0; j<height; j++){
            cloud->points[j*width+i].x = (float(i)-cx)*cloud->points[j*width+i].z/fx;
            cloud->points[j*width+i].y = (float(j)-cy)*cloud->points[j*width+i].z/fy;
        }
    }
}

void CameraTracker::checkAngles(Vector6f &vec)
{
   for(size_t i=3; i<6; i++){
       while (vec(i)>M_PI)  {vec(i) -= 2*M_PI;}
       while (vec(i)<-M_PI) {vec(i) += 2*M_PI;}
   }
}

Eigen::Affine3f CameraTracker::toEigen(Vector6f pose)
{
    return pcl::getTransformation(pose(0),pose(1),pose(2),pose(3),pose(4),pose(5));
}

Vector6f CameraTracker::toVector(Eigen::Affine3f pose)
{
    Vector6f temp;
    pcl::getTranslationAndEulerAngles(pose, temp(0,0),temp(1,0),temp(2,0),temp(3,0),temp(4,0),temp(5,0));
    checkAngles(temp);
    return temp;
}

CloudPtr CameraTracker::Mat2Cloud(Mat &imR, Mat &imD)
{
    CloudPtr cloud;
    cloud.reset(new Cloud);
    cloud->points.resize(width*height);

        for(size_t j=0; j<height; j++){
            for(size_t i =0; i < width; i++){
            PointT pt;
            if (! (float(imD.ptr<ushort>(j)[i])==float(imD.ptr<ushort>(j)[i]))) {
                pt.z= 0.f/0.f;
                cloud->points.at(j*width+i) = pt;
                continue;
                cout << "failed" <<endl;
            }
            pt.z = float(imD.ptr<ushort>(j)[i])/10000;
          //  cout << "the value of point's depth " << pt.z << endl;
            pt.x = (float(i) - cx) * pt.z / fx;
            pt.y = (float(j) - cy) * pt.z / fy;
            Vec3b color = imR.at<Vec3b>(j,i);
            pt.r = (int)color.val[0];
            pt.g = (int)color.val[1];
            pt.b = (int)color.val[2];
            cloud->points.at(j*width+i) = pt;
        }
    }
    return cloud;
}

void CameraTracker::sampleForegroundEdge(CloudPtr& m_src, Frame::Ptr pFrame)
{
    cout<< m_src->points.size() << endl;
    int cnt=0;
    cout<<width<<"  "<<height<<endl;
    for(size_t j=4; j<height-4; j++){
        for(size_t i=4; i<width-4; i++){
            PointT pt_src=m_src->points[j*width+i];

            if (pt_src.z!=pt_src.z || pt_src.z>8.f) {continue;}  //continue if no depth value available

            //warp to target image
            PointT ptwarp=pt_src;
            ptwarp= pcl::transformPoint(ptwarp, m_key2cur);
            int xpos = int(floor(fx/ptwarp.z * ptwarp.x + cx));
            int ypos = int(floor(fy/ptwarp.z * ptwarp.y + cy));
            if (xpos>=width-4 || ypos>=height-4 || xpos<4 || ypos<4) {continue;} //continue if out of image border


            //check whether foreground point in src
            float z_ = m_src->points[j*width+i].z;
            float diff1=z_ - m_src->points[j*width+i+4].z;
            float diff2=z_ - m_src->points[j*width+i-4].z;
            float diff3=z_ - m_src->points[(j-4)*width+i].z;
            float diff4=z_ - m_src->points[(j+4)*width+i].z;
            if ( diff1!=diff1 || diff2!=diff2 || diff3!=diff3 || diff4!=diff4){
                                continue;
            }
      //      float thres= 0.021*z_;
            float Tb = 0.015,Tf = 0.05;
            if((max_4(diff1,diff2,diff3,diff4) < (z_ * Tb)) && (max(fabs(diff1 - diff2),fabs(diff3 - diff4)) > (z_*Tf)))
            {
                pFrame->ForegroundEdge_.insert(make_pair(cv::Point2i(i,j),m_src->points[j*width+i]));
                pFrame->ForegroundEdgePix.push_back(cv::Point2i(i,j));
                cnt++;
                continue;
            }
        }
    }

    cout<<"sampled "<< cnt<<" foreground edge points"<<endl;
}

void CameraTracker::run(Frame::Ptr pFrame)   //feed in rgb and depth image
{
    CloudPtr scene (new Cloud());
    scene = Mat2Cloud(pFrame->color_, pFrame->depth_);
  //  pcl::isualization::CloudViewer viewer("pcd viewer");
 //   viewer.showCloud(scene);
    sampleForegroundEdge(scene,pFrame);
    if(pFrame->ForegroundEdge_.size() == 0)
        return;
    Mat test(cv::Size(640,480),CV_8U,cv::Scalar(0));
    for(int i = 0; i < pFrame->ForegroundEdgePix.size(); i++)
    {
        test.at<uchar>(pFrame->ForegroundEdgePix[i].y,pFrame->ForegroundEdgePix[i].x) = 255;
    }
    cv::imshow("test",test);
    cv::waitKey(30);
    if (m_cnt==0) {  //for the first frame data
     //   sampleForegroundEdge(scene,pFrame);
        icpEst.setupSource(pFrame->ForegroundEdge_,pFrame->ForegroundEdgePix);
        pFrame->setPose(Eigen::Affine3f::Identity());
        MyMapPtr->insertKeyFrame(pFrame);
        currentKeyFrame = pFrame;
        m_cnt++;
        return;
    //    m_current = scene;
    }

    else{
     //   Vector6f temp;
      //  temp = toVector(m_key2cur);
     //   checkAngles(temp);
        //change keyframe if key2current is greater than certain threshold
     //   float dthr=0.07f, athr = (8.f/180)*M_PI;
      //  if((sqrtf(pow(temp(0),2)+ pow(temp(1),2)+ pow(temp(2),2))> dthr|| sqrtf(pow(temp(3),2) + pow(temp(4),2) + pow(temp(5),2))>athr)){
        //change key frame very  nth frame
        if(m_cnt%5==0){
            currentKeyFrame = lastFrame;
            m_changeKey=false;
            m_key2cur=Eigen::Affine3f::Identity();
            currentKeyFrame->setPose(m_pose);
            m_keypose = m_pose;
            icpEst.setupSource(currentKeyFrame->ForegroundEdge_,currentKeyFrame->ForegroundEdgePix);
            MyMapPtr->insertKeyFrame(currentKeyFrame,matchedPoints);
        }
    }

    m_lasttime = m_thistime;
    lastFrame = pFrame;
    m_thistime = pFrame->time_stamp_;
    //estimate speed/acc
    m_lastspeed = m_speed;
    double dtime = m_thistime-m_lasttime;
    if (dtime< 1e6f*0.02f) {dtime=1e6f *0.03f;}
    m_speed= toVector(m_pose * m_lastpose.inverse());
    checkAngles(m_speed); m_speed = (m_speed/dtime) /**0.5f + m_lastspeed*0.5f*/;
    m_acc = (m_speed-m_lastspeed);
    checkAngles(m_acc);  m_acc = (m_acc/dtime);

    cout<<"time interval is "<<m_thistime-m_lasttime<<endl;

    m_cnt++;
/*
    //predict key2cur pose
    dtime = m_thistime-m_lasttime;
    if (dtime> 1e6f*0.04f) {dtime=1e6f *0.04f;}
    if(m_cnt>12){
        m_key2cur = toEigen(toVector(toEigen((m_speed*dtime + 0.5f*m_acc*pow(dtime, 2)) * 0.99f) * m_key2cur));
    }
    */
    // sampleForegroundEdge(m_current,pFrame);
    icpEst.setupTarget(pFrame->ForegroundEdge_,pFrame->ForegroundEdgePix);
    icpEst.setupPredict_(m_key2cur);
    icpEst.run(currentKeyFrame->ForegroundEdge_Ws);
    icpEst.getCorrespondenceAndDist(geoResdiuals,geoResdiuals_all,matchedPoints);
    cout << "the size of matched Points " << matchedPoints.size() << endl;
 //   m_key2cur =  icpEst.getTransResult();
    m_lastpose=m_pose;
    m_pose = toEigen(toVector(m_key2cur*m_keypose));
    currentKeyFrame->UpdateStaticWeight(geoResdiuals,geoResdiuals_all,matchedPoints,pFrame->id_);
    Mat weightshow(cv::Size(640,480),CV_8UC3,cv::Scalar(0));
    vector<float> currentKeyFrame_ws;
    for(int i = 0; i < currentKeyFrame->ForegroundEdgePix.size(); i++)
    {
        currentKeyFrame_ws.push_back(currentKeyFrame->ForegroundEdge_Ws[currentKeyFrame->ForegroundEdgePix[i]]);
    }
    sort(currentKeyFrame_ws.begin(), currentKeyFrame_ws.end());
    for(int i = 0; i < currentKeyFrame->ForegroundEdgePix.size(); i++)
    {
        if(currentKeyFrame->ForegroundEdge_Ws[currentKeyFrame->ForegroundEdgePix[i]] > currentKeyFrame_ws[currentKeyFrame_ws.size()/2])
            weightshow.at<Vec3b>(currentKeyFrame->ForegroundEdgePix[i].y,currentKeyFrame->ForegroundEdgePix[i].x) = Vec3b(0,255,0);
        else {
             weightshow.at<Vec3b>(currentKeyFrame->ForegroundEdgePix[i].y,currentKeyFrame->ForegroundEdgePix[i].x) = Vec3b(0,0,255);

        }

    }
    cv::imshow("weigth end show",weightshow);
    cv::waitKey(30);


//    if (m_cnt==1){
//        m_key2cur =  Affine3f::Identity();

//        m_lastpose=Affine3f::Identity();
//        m_pose = Affine3f::Identity();

//    }

   // cout<<"current camera pose realtive to first frame is: "<<endl<<m_pose.matrix()<<endl;
    //mapping : store frame every 20th frame
    /*
    if (m_cnt%30==1){
        CloudPtr temp(new Cloud());
        for(size_t i=0; i<width/8; i++){
            for(size_t j=0; j<height/8; j++){
                temp->points.push_back(m_current->points[j*8*width+i*8]);
            }
        }
        transformPointCloud(*temp, *temp, m_pose.inverse());
        *m_map += *temp;
    }
    */
}

float CameraTracker::max_4(float diff1, float diff2, float diff3, float diff4)
{
    float Max = diff1;
    Max = max(Max,diff2);
    Max = max(Max,diff3);
    Max = max(Max,diff4);
    return Max;
}
