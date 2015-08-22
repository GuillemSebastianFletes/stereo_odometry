#include <iostream>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/video/tracking.hpp"



using namespace cv;
using namespace std;



class stereo_odometry
{
    //methods before args.
private:

public:

    stereo_odometry();
    ~stereo_odometry();
    void init();
    void translation( Mat &actual_frame, Mat &previous_frame);
    //std_msgs::Float64 translation_;
    //std_msgs::Float64 rotation_;
};
