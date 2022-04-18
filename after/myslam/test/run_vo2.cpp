#include "myslam/visual_odometry.h"
#include "opencv4/opencv2/imgcodecs.hpp"
bool camera_init=false;
 myslam::Camera::Ptr camera;
 myslam::VisualOdometry::Ptr vo;
 myslam::testPoint::Ptr tp;
int readImg( string  color,string depth){
    if(!camera_init){
       camera=make_shared<myslam::Camera>(myslam::Camera());
        vo=make_shared<myslam::VisualOdometry>(myslam::VisualOdometry());
        tp=make_shared<myslam::testPoint>(myslam::testPoint());

    }

    myslam::Frame::Ptr pFrame=myslam::Frame::createFrame();

    Mat color_mat=cv::imread(color);
    Mat depth_mat=cv::imread(depth);
    pFrame->camera_=camera;
    pFrame->color_=color_mat;
    pFrame->depth_=depth_mat;
    

    vo->addFrame(pFrame);
    if(vo->state_==myslam::VisualOdometry::LOST){
        return 1;
    }




}