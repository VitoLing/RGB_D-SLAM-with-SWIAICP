#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
//#include <opencv2/viz/viz3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "config.h"
#include "iaicp.h"
#include "camera_tracker.h"
#include "camera.h"
#include "frame.h"

//CameraTracker theTracker;
int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    cout << "hello world" << endl;
    cout << argv[1] << endl;
    Config::setParameterFile ( argv[1] );
    cout << endl << "Reading: " << endl;
    cv::FileStorage fs;
    fs.open(argv[1], cv::FileStorage::READ);
//    string dataset_dir = fs["dataset_dir"];
//    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
    string dataset_dir = Config::get<string>("dataset_dir");
    CameraTracker theTracker ;
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }
    Camera::Ptr camera ( new Camera );
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        if(i == 242 || i == 724)
            continue;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );

        if ( color.data==nullptr || depth.data==nullptr )
            break;
        Frame::Ptr pFrame = Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];
       // pFrame->RGB2GRAY();

        /*
        Mat image;
        imshow("gray:", pFrame->intensity_);
        cv::threshold(pFrame->intensity_,image,0,255,cv::THRESH_BINARY);
        cv::GaussianBlur(pFrame->intensity_, image,Size(3,3),0);
        cv::Canny(image,image,100,250);
        std::vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(image,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());
        Mat imageContours = Mat::zeros(image.size(),CV_8UC1);
        Mat Contours = Mat::zeros(image.size(),CV_8UC1);
        for(int i = 0; i < contours.size();i++)
        {
            for(int j = 0; j < contours[i].size(); j++)
            {
                Point P  = cv::Point(contours[i][j].x,contours[i][j].y);
                Contours.at<uchar>(P) = 255;
            }
            // output the content of hierarchy
            char ch[256];
            sprintf(ch,"%d",i);
            string str = ch;
            cout << "vector hierarchy's " << str << "element" << endl << hierarchy[i] << endl << endl;

            // draw contours
            cv::drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);
        }
        imshow("Contours image",imageContours); // contours
        imshow("Point of Contours",Contours);
        waitKey(20);
        */
        boost::timer timer;
        theTracker.run(pFrame);
        cout<<"VO costs time: "<<timer.elapsed() <<endl;
        //SE3 Twc = pFrame->T_c_w_.inverse();
        Mat img_show = color.clone();
        Mat depth_show = depth.clone();
     //   cv::imshow ( "image", img_show );
        // cv::imshow("depth",depth_show);
        //cv::imshow("test",test);
     //   cv::waitKey(30);
    }
    return 0;
}
