#include  "run_vo2.h"
bool camera_init = false;

myslam::Camera::Ptr camera;
myslam::VisualOdometry::Ptr vo;
myslam::Frame::Ptr pFrame;
string readImg(string color,string depth)
{

    

    if (!camera_init)
    {
         myslam::Config::setParameterFile ( "/home/guojiawei/cv/myslam/config/default.yaml" );
        camera = make_shared<myslam::Camera>(myslam::Camera());
        vo = make_shared<myslam::VisualOdometry>(myslam::VisualOdometry());
        
        camera_init = true;
    }

    Mat color_mat = cv::imread(color);
    Mat depth_mat = cv::imread(depth);
    myslam::Frame::Ptr pFrame=myslam::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->color_ = color_mat;
    pFrame->depth_ = depth_mat;

    // cout << argc << endl;

    vo->addFrame(pFrame);

    if (vo->state_ == myslam::VisualOdometry::LOST)
    {
        cout<<"!"<<endl;
        return "!";
    }

vector<double> v;
    
vo->getAllPoints(v);
string ans;
for(size_t i=0;i<v.size();i++){
    ans=ans+"\t"+std::to_string(v[i]);
}
cout<<"size="<<v.size()<<endl;
return ans;



}
void changeInit(){


    camera_init=false;
}