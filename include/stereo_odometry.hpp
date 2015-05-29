#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cmath>
#include <math.h>
#include <std_msgs/Float64.h>


using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;


class stereo_odometry
{
    //methods before args.
private:

    bool first_execution_;
    Mat prev_image_left_;
    Mat prev_image_right_;

public:

    stereo_odometry();
    ~stereo_odometry();
    void init();
    void calculus( Mat &left_image, Mat &right_image);
    std_msgs::Float64 translation_;
    std_msgs::Float64 rotation_;
};
