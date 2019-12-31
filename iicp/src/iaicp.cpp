#include "iaicp.h"
#include <Eigen/Geometry>
#include "config.h"
using namespace pcl;


Iaicp::Iaicp()
{
    fx = Config::get<float>("/fx");
    fy = Config::get<float>("/fy");
    cx = Config::get<float>("/cx");
    cy = Config::get<float>("/cy");
    height = Config::get<float>("/height");
    width = Config::get<float>("/width");
    m_trans=Affine3f::Identity();
    m_predict=Affine3f::Identity();
}

Iaicp::~Iaicp(){}

void Iaicp::setupSource(ForegroundEdgeMap &source, vector<cv::Point2i>& sourcePix)
{
    m_ForegroundSrcPixel.clear();
    m_ForegroundSrcPixel = sourcePix;
    m_ForegroundSrc.clear();
    m_ForegroundSrc = source;
    intMedian=0.f;  intMad=45.f;
    cout << "source set success" << endl;
}

void Iaicp::setupTarget(ForegroundEdgeMap &target, vector<Point2i> &targetPix)
{
    m_ForegroundTgtPixel.clear();
    m_ForegroundTgtPixel = targetPix;
    m_ForegroundTgt.clear();
    m_ForegroundTgt = target;
    geoMedian =0.f; geoMad=0.02f;
    intMedian=0.f;  intMad=45.f;
    cout << "target set success" << endl;
}

void Iaicp::setupPredict_(Eigen::Affine3f pred)
{
    cout << "start  set Predict" << endl;
    m_predict = pred;
    m_trans = pred;
    cout << "prediect set success" << endl;
}


void Iaicp::run(StaticWeightMap &SourceStaticWeight)
{
    int iterPerLevel = 7;
    int offset=1, maxDist=0.25f;
    iterateLevel(maxDist, offset, iterPerLevel,SourceStaticWeight);
    offset=1; maxDist=0.06f;
    iterateLevel(maxDist, offset, iterPerLevel, SourceStaticWeight);
    offset=1; maxDist=0.02f;
    iterateLevel(maxDist, offset,15,SourceStaticWeight);
}

void Iaicp::iterateLevel(float maxDist, int offset, int maxiter,StaticWeightMap &SourceStaticWeight) //performs one iteration of the IaICP method
{
    for (size_t iteration=0; iteration<maxiter; iteration++){
        tgt_.reset(new Cloud());
        src_.reset(new Cloud());
        std::vector<float> geoResiduals;
        std::vector<float> intResiduals;
        std::vector<float> staticweights;

        int counter=0;   //counter for how many number of correspondences have been already used

        for(int i = 0; i < m_ForegroundSrcPixel.size(); i++){
            bool flag = false; // whether find the correspondence point
            if (counter>= 1200) break;    //only use  limited number of pairs of correspondences.
            int thisIndex =  rand()%m_ForegroundSrc.size();  //randomly select one foreground point
            PointT temp = m_ForegroundSrc[m_ForegroundSrcPixel[thisIndex]];   //selected source ponint
            PointT pt=transformPoint(temp, m_trans);   //warped source point
            PointT tgtpt;                              //for the selected correponding point from the target cloud.
            int xpos = int(floor(fx/pt.z * pt.x + cx)); //warped image coordinate x
            int ypos = int(floor(fy/pt.z * pt.y + cy)); //warped image coordinate y

            if (xpos>=(width) || ypos>=(height)|| xpos<0 || ypos<0) { continue;}
            float maxWeight = 1e-10;
            int searchRange=4;
            float intResidual, geoResidual;
            for(int xx=-searchRange; xx<searchRange+1; xx++){
                for(int yy=-searchRange; yy<searchRange+1; yy++){
                    float gridDist = sqrt(pow(float(xx),2) + pow(float(yy),2));
                    if ( gridDist > (float)searchRange ){continue;}  //get a circle shaped search area
                    int xpos_ = xpos+xx*(float)offset;  //searched target point's image coordinate
                    int ypos_ = ypos+yy*(float)offset;
                    if (xpos_>=(width-4) || ypos_>=(height-4) || xpos_<4 || ypos_<4) { continue;}
                    auto got = m_ForegroundTgt.find(cv::Point2i(xpos,ypos));
                    if(got == m_ForegroundTgt.end())
                        continue;

                    PointT pt2 = got->second;

                    float dist = (pt.getVector3fMap()-pt2.getVector3fMap()).norm();  //geo. distance
                    if(dist==dist){           //check for NAN
             //           if (dist>maxDist) {continue;}
                        float residual = getresidual(pt2, pt);
                        flag = true;
                        if(residual==residual){  //check for NAN
                            float geoWeight = 1e2f*(6.f/(5.f+ pow((dist)/(geoMad), 2)));
                            float colWeight = 1e2f*(6.f/(5.f+ pow((residual-intMedian)/intMad, 2)));
                            float thisweight = geoWeight * colWeight;
                            if(thisweight==thisweight && thisweight>maxWeight){
                                tgtpt=pt2;
                                maxWeight=thisweight;
                                intResidual= residual; geoResidual = dist;
                            }
                        }
                    }
                }
            }

            if(maxWeight>0 && flag == true ){
                if ((m_ForegroundSrc[m_ForegroundSrcPixel[thisIndex]].getVector3fMap()-tgtpt.getVector3fMap()).norm()<1000.f){
                     src_->points.push_back(pt);
                     tgt_->points.push_back(tgtpt);

                     intResidual=getresidual(tgtpt, pt);
                     geoResidual = (pt.getVector3fMap()-tgtpt.getVector3fMap()).norm();
                     intResiduals.push_back(intResidual);
                     geoResiduals.push_back(geoResidual);
                     staticweights.push_back(SourceStaticWeight[m_ForegroundSrcPixel[thisIndex]]);
                     counter++;
                }
            }
        }

        //Estimate median and deviation for both intensity and geometry residuals
        vector<float> temp = geoResiduals;
        sort(temp.begin(), temp.end());
        geoMedian = temp[temp.size()-temp.size()/2];
        for(size_t i=0; i<temp.size(); i++){
            temp[i] = fabs(temp[i]-geoMedian);
        }
        sort(temp.begin(), temp.end());
        geoMad = 1.f*1.4826 * temp[temp.size()/2]+1e-11;
        for(size_t i=0; i<geoResiduals.size(); i++){
            geoResiduals[i] =  (6.f/(5.f+ pow((geoResiduals[i])/geoMad, 2)));
        }
        temp.clear();
        temp = intResiduals;
        sort(temp.begin(), temp.end());
        intMedian = temp[temp.size()-temp.size()/2];
        for(size_t i=0; i<temp.size(); i++){
            temp[i] = fabs(temp[i]-intMedian);
        }
        sort(temp.begin(), temp.end());
        intMad = 1.f*1.4826 * temp[temp.size()/2]+1e-11;
        for(size_t i=0; i<intResiduals.size(); i++){
            intResiduals[i] = (6.f/(5.f+ pow((intResiduals[i]-intMedian)/intMad, 2)));
        }

        pcl::TransformationFromCorrespondences transFromCorr;
        for (size_t i =0;i<src_->points.size();i++)
        {
                        Eigen::Vector3f from(src_->points.at(i).x, src_->points.at(i).y, src_->points.at(i).z);
                        Eigen::Vector3f to(tgt_->points.at(i).x, tgt_->points.at(i).y, tgt_->points.at(i).z);
                        float sensorRel = 1.f/(0.0012+0.0019*pow(src_->points.at(i).z-0.4, 2));
                        transFromCorr.add(from, to, geoResiduals[i] * intResiduals[i]*sensorRel*staticweights[i]);

        }
        Eigen::Affine3f increTrans= transFromCorr.getTransformation();
        m_trans = toEigen(toVector(increTrans *m_trans) ) ;
    }
}

void Iaicp::getCorrespondenceAndDist(vector<float>& geoResiduals,vector<float>& geoResiduals_all, MatchedPointMap & MatchedPoint_)
{
    tgt_.reset(new Cloud());
    src_.reset(new Cloud());
    geoResiduals_all.clear();
    geoResiduals.clear();
    MatchedPoint_.clear();
    int offset = 1;

    for(int i = 0; i < m_ForegroundSrcPixel.size(); ++i){
        bool flag = false; // whether find the correspondence point
        PointT temp = m_ForegroundSrc[m_ForegroundSrcPixel[i]];   //selected source ponint
        PointT pt=transformPoint(temp, m_trans);   //warped source point
        PointT tgtpt;                              //for the selected correponding point from the target cloud.
        cv::Point2i  tgtPix;
        int xpos = int(floor(fx/pt.z * pt.x + cx)); //warped image coordinate x
        int ypos = int(floor(fy/pt.z * pt.y + cy)); //warped image coordinate y

        if (xpos>=(width) || ypos>=(height)|| xpos<0 || ypos<0) { continue;}
        float maxWeight = 1e-10;
        int searchRange=10;
        float intResidual, geoResidual;
        for(int xx=-searchRange; xx<searchRange+1; xx++){
            for(int yy=-searchRange; yy<searchRange+1; yy++){
                float gridDist = sqrt(pow(float(xx),2) + pow(float(yy),2));
                if ( gridDist > (float)searchRange ){continue;}  //get a circle shaped search area
                int xpos_ = xpos+xx*(float)offset;  //searched target point's image coordinate
                int ypos_ = ypos+yy*(float)offset;
                if (xpos_>=(width-4) || ypos_>=(height-4) || xpos_<4 || ypos_<4) { continue;}
                auto got = m_ForegroundTgt.find(cv::Point2i(xpos,ypos));
                if(got == m_ForegroundTgt.end())
                    continue;

                PointT pt2 = got->second;

                float dist = (pt.getVector3fMap()-pt2.getVector3fMap()).norm();  //geo. distance
                if(dist==dist){           //check for NAN
//                        if (dist>maxDist) {continue;}
                    float residual = getresidual(pt2, pt);
                    flag = true;
                    if(residual==residual){  //check for NAN
                        float geoWeight = 1e2f*(6.f/(5.f+ pow((dist)/(geoMad), 2)));
                        float colWeight = 1e2f*(6.f/(5.f+ pow((residual-intMedian)/intMad, 2)));
                        float thisweight = geoWeight * colWeight;
                        if(thisweight==thisweight && thisweight>maxWeight){
                            tgtpt=pt2;
                            maxWeight=thisweight;
                            geoResidual = dist;
                            tgtPix = got->first;
                        }
                    }
                }
            }
        }

        if(maxWeight>0 && flag == true ){
            if ((temp.getVector3fMap()-tgtpt.getVector3fMap()).norm()<1000.f){
                 src_->points.push_back(pt);
                 tgt_->points.push_back(tgtpt);
                 MatchedPoint_.insert(make_pair(m_ForegroundSrcPixel[i],tgtPix));
                 geoResidual = (pt.getVector3fMap()-tgtpt.getVector3fMap()).norm();
                 geoResiduals.push_back(geoResidual);
                 geoResiduals_all.push_back(geoResidual);
            }
        }
        else {
            geoResiduals_all.push_back(1000.f);
        }
    }
}

void Iaicp::checkAngles(Vector6f &vec)
{
   for(size_t i=3; i<6; i++){
       while (vec(i)>M_PI)  {vec(i) -= 2*M_PI;}
       while (vec(i)<-M_PI) {vec(i) += 2*M_PI;}
   }
}

Eigen::Affine3f Iaicp::toEigen(Vector6f pose)
{
    return pcl::getTransformation(pose(0),pose(1),pose(2),pose(3),pose(4),pose(5));
}

Vector6f Iaicp::toVector(Eigen::Affine3f pose)
{
    Vector6f temp;
    pcl::getTranslationAndEulerAngles(pose, temp(0),temp(1),temp(2),temp(3),temp(4),temp(5));
    checkAngles(temp);
    return temp;
}


