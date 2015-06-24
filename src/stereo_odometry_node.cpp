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

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    ///execution time variables
    std::clock_t start;
    double duration;

    ///path variables
    char path[] = "/home/guillem/07/image_0/";
    char format[] = ".png";
    char name[50];
    char name_old[50];
    int n;

    ///images initialitation
    Mat actual_frame;
    Mat previous_frame;
    Mat output_actual;
    Mat output_old;

    ///SIFT algorith variables
    vector<cv::KeyPoint> keypoints_actual;
    vector<cv::KeyPoint> keypoints_old;
    vector<cv::KeyPoint> good_keypoints_actual;
    vector<cv::KeyPoint> good_keypoints_old;
    int nfeatures=0;
    int nOctaveLayers=3;
    double contrastThreshold=0.09;
    double edgeThreshold=10;
    double sigma=1.6;

    SiftFeatureDetector detector (nfeatures, nOctaveLayers, contrastThreshold,
                                  edgeThreshold, sigma);
    SiftDescriptorExtractor extractor;

    ///Matching algorithm variables
    Mat descriptors_actual,descriptors_old;
    BruteForceMatcher<L2 <float> > matcher;
    vector<cv::DMatch> matches;


    ///movement points elimination variables
    std::vector<double> distance;
    std::vector<double> angle;
    distance.clear();
    double media_distance;
    double media_angle;
    int contador;

    //actual keypoints
    float x1;
    float y1;

    //old keypoints
    float x2;
    float y2;

    //final vestor
    double x;
    double y;

    for (int i = 1; i < 1101; i++)
    {
        start = std::clock();

        ///image name initialitation
        n = sprintf(name,"%s%.6d%s", path, i, format);
        n = sprintf(name_old,"%s%.6d%s",path,i-1,format);
        // cout << name << "   " << name_old << endl;

        ///Reading the images
        actual_frame = imread(name);
        previous_frame = imread(name_old);

        ///Translation
        //ROI
        actual_frame = actual_frame(Rect(0,1*actual_frame.rows/3,
                                         actual_frame.cols, 2*actual_frame.rows/3));
        previous_frame = previous_frame(Rect(0,1*previous_frame.rows/3,
                                             previous_frame.cols, 2*previous_frame.rows/3));

        //SIFT detection
        detector.detect(actual_frame,keypoints_actual);
        detector.detect(previous_frame,keypoints_old);

        //Matching characteristics
        extractor.compute(actual_frame,keypoints_actual,descriptors_actual);
        extractor.compute(previous_frame,keypoints_old,descriptors_old);
        matcher.match(descriptors_actual,descriptors_old,matches);


        angle.clear();
        distance.clear();
        media_distance = 0;
        media_angle = 0;
        contador = 0;

        std::vector<int8_t> it_counter_actual;
        std::vector<int8_t> it_counter_old;

        //Getting points with matches between frames
        for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it)
        {
            // Get the position of actual keypoints
            x1= keypoints_actual[it->queryIdx].pt.x;
            y1= keypoints_actual[it->queryIdx].pt.y;

            // Get the position of old keypoints
            x2= keypoints_old[it->trainIdx].pt.x;
            y2= keypoints_old[it->trainIdx].pt.y;

            //final vector
            x = x2-x1;
            y = y2-y1;

            distance.push_back(sqrt(x*x+y*y));
            angle.push_back((atan2(y,x)*180 / PI)*(-1));

            //Computing the media

            media_distance = media_distance + sqrt(x*x+y*y);
            media_angle = media_angle + ((atan2(y,x)*180 / PI)*(-1));
            it_counter_actual.push_back(it->queryIdx);
            it_counter_old.push_back(it->trainIdx);
            contador ++;
        }

        media_distance = media_distance/contador;
        media_angle = media_angle/contador;

        if (distance.empty())
        {
            cout << "fail"<<endl;
        }

        else
        {
            cout << "success"<<endl;
        }


        //if distance[i] && angle[i] belongs to the working area add the points to the new keypoint actual and new
        if(it_counter_actual.size() == it_counter_old.size())
        {
            for (int i =0; i!=it_counter_actual.size(); i++)
            {
                if(distance.at(i) > 0.5 * media_distance && distance.at(i) < 1.5 * media_distance)
                {
                    if(angle.at(i) > 0.5 * media_angle && angle.at(i) < 1.5 * media_angle)
                    {

                        good_keypoints_actual.push_back(keypoints_actual.at(it_counter_actual.at(i)));

                        good_keypoints_old.push_back(keypoints_old.at(it_counter_old.at(i)));
                    }
                }
            }
        }

        ///visualitation
        //drawing the feature's points
        drawKeypoints(actual_frame, good_keypoints_actual, output_actual);
        drawKeypoints(previous_frame, good_keypoints_old, output_old);
        imshow("actual", output_actual);
        imshow("old", output_old);
        waitKey(1);

        keypoints_actual.clear();
        keypoints_old.clear();
        descriptors_actual.release();
        descriptors_old.release();
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"duration: "<< duration <<'\n';
    }

    return 0;
}
