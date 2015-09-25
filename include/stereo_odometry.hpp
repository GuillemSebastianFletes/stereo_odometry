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
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/video/tracking.hpp"
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/point_types.h>



using namespace cv;
using namespace std;



class stereo_odometry
{
    //methods before args.
private:

    void get_good_points(Mat &actual_frame_l, Mat &previous_frame_l);
    void compute_disparity_map(Mat &actual_frame_l, Mat &previous_frame_l, Mat &actual_frame_r, Mat &previous_frame_r);
    vector<cv::KeyPoint> keypoints_old;
    vector<cv::KeyPoint> good_keypoints_actual;
    vector<cv::KeyPoint> good_keypoints_old;
    bool first_iteration;
    bool first_iteration_disparity;
    Mat disparity_old;
    Mat disparity_actual;

public:

    stereo_odometry();
    ~stereo_odometry();
    void init();
    void translation( Mat &actual_frame_l, Mat &previous_frame_l, Mat &actual_frame_r, Mat &previous_frame_r);
    //std_msgs::Float64 translation_;
    //std_msgs::Float64 rotation_;
};
