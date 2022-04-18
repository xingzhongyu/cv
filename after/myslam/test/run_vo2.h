#ifndef RUN_H
#define RUN_H
#include  "myslam/visual_odometry.h"
#include "myslam/config.h"
#include "opencv4/opencv2/imgcodecs.hpp"
string readImg(string color,string depth);

void changeInit();
#endif