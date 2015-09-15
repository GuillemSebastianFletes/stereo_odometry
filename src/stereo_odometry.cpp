#include <stereo_odometry.hpp>

using namespace cv;
using namespace std;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816

//constructor
stereo_odometry::stereo_odometry(){}
stereo_odometry::~stereo_odometry(){}
//stereo_odometry::init(){}

///Translation

void stereo_odometry::translation(Mat &actual_frame, Mat &previous_frame)
{

    ///images initialitation
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
    double media_distance = 0;
    double media_angle = 0;
    int contador;

    //actual keypoints
    float x1;
    float y1;

    //old keypoints
    float x0;
    float y0;

    //final vestor
    double x;
    double y;

    std::vector<int> it_counter_actual;
    std::vector<int> it_counter_old;



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


    //Getting points with matches between frames
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it)
    {
        // Get the position of actual keypoints
        x1= keypoints_actual[it->queryIdx].pt.x;
        y1= keypoints_actual[it->queryIdx].pt.y;

        // Get the position of old keypoints
        x0= keypoints_old[it->trainIdx].pt.x;
        y0= keypoints_old[it->trainIdx].pt.y;

        //final vector
        x = x1-x0;
        y = y1-y0;

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

    if (it_counter_actual.empty() || it_counter_old.empty())
    {
        cout << "fail" << endl;
        return;
    }

    else
    {
        cout << "success"<<endl;
    }

    /*if distance[i] && angle[i] belong to the working area add the points to the new keypoint actual and new
    it is necessary to take in account that keypoints_old/new haven't got the same size*/

        for (int i =0; i < contador; i++)
        {
            if(distance.at(i) > 0.85 * media_distance && distance.at(i) < 1.15 * media_distance)
            {
                if(angle.at(i) > 0.85 * media_angle && angle.at(i) < 1.15 * media_angle)
                {
                    good_keypoints_actual.push_back(keypoints_actual[it_counter_actual[i]]);
                    good_keypoints_old.push_back(keypoints_old[it_counter_old[i]]);

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

}
