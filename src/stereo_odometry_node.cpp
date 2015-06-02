#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <cmath>
#include <math.h>
//#include <std_msgs/Float64.h>


using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    char path[] = "/home/guillem/07/image_0/";
    char format[] = ".png";
    char name[50];
    char name_old[50];
    int n;

    Mat actual_frame;
    Mat previous_frame;

    for (int i = 1; i < 1101; i++) //cargar la secuencia imagenes
    {

        n = sprintf(name,"%s%.6d%s", path, i, format);
        n = sprintf(name_old,"%s%.6d%s",path,i-1,format);
        cout << name << "   " << name_old << endl;


        actual_frame = imread(name);
        previous_frame = imread(name_old);
        namedWindow( "actual", WINDOW_AUTOSIZE);
        namedWindow( "old", WINDOW_AUTOSIZE);
        imshow("actual", actual_frame);
        imshow("old",previous_frame);
        waitKey(10);
    }

    return 0;
}
