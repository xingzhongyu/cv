#include  <opencv4/opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
using namespace cv;
using namespace std;
int main(){
    boost::format fmt("/home/guojiawei//cv/cv/after/data/%s/%d.%s"); //图像文件格式
    Mat img=cv::imread((fmt % "depth" % (1) % "pgm").str(), -1);
    cout<<img<<endl;

    
    return 0;
}