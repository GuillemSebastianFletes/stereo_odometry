#include <stereo_odometry.hpp>

using namespace cv;
using namespace std;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816

//constructor
stereo_odometry::stereo_odometry()
{
    first_iteration = true;
    first_iteration_disparity = true;
}
stereo_odometry::~stereo_odometry(){}
//stereo_odometry::init(){}

///Translation

void stereo_odometry::translation(Mat &actual_frame_l, Mat &previous_frame_l, Mat &actual_frame_r, Mat &previous_frame_r)
{
    //ROI
    actual_frame_l = actual_frame_l(Rect(0,1*actual_frame_l.rows/3,
                                         actual_frame_l.cols, 2*actual_frame_l.rows/3));
    previous_frame_l = previous_frame_l(Rect(0,1*previous_frame_l.rows/3,
                                             previous_frame_l.cols, 2*previous_frame_l.rows/3));

    actual_frame_r = actual_frame_r(Rect(0,1*actual_frame_r.rows/3,
                                         actual_frame_r.cols, 2*actual_frame_r.rows/3));
    previous_frame_r = previous_frame_r(Rect(0,1*previous_frame_r.rows/3,
                                             previous_frame_r.cols, 2*previous_frame_r.rows/3));


    //Get the key points
    // get_good_points(actual_frame_l, previous_frame_l);



    //Compute disparity map
    compute_disparity_map(actual_frame_l, previous_frame_l, actual_frame_r, previous_frame_r);

    disparity_old = disparity_actual;
}

void stereo_odometry::compute_disparity_map(Mat &actual_frame_l, Mat &previous_frame_l, Mat &actual_frame_r, Mat &previous_frame_r)
{
    //SGBM variables initalitation

    int minDisparity = 0;
    int numDisparities = 80;
    int SADWindowSize = 1;
    int preFilterCap =0;
    int uniquenessRatio = 0;
    int P1 = 0;
    int P2 = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int disp12MaxDiff = 0;
    bool fullDP = false;

    static StereoSGBM mystereoSGBM( minDisparity, numDisparities, SADWindowSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, fullDP);

    mystereoSGBM(actual_frame_l, actual_frame_r, disparity_actual);


    if (first_iteration_disparity)
    {
        mystereoSGBM(previous_frame_l, previous_frame_r, disparity_old);
    }

    ///Test
    imshow("actual disparity", disparity_actual);
    imshow("previous disparity", disparity_old);
    waitKey(1);

}


void stereo_odometry::get_good_points(Mat &actual_frame_l, Mat &previous_frame_l)
{
    ///images initialitation
    Mat output_actual;
    Mat output_old;


    ///SIFT algorith variables
    vector<cv::KeyPoint> keypoints_actual;
    //    vector<cv::KeyPoint> keypoints_old;
    int nfeatures=0;
    int nOctaveLayers=3;
    double contrastThreshold=0.01;//0.09
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
    double angle_prov;
    double media_distance = 0;
    double media_angle = 0;
    int contador = 0;

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

    //SIFT detection
    detector.detect(actual_frame_l,keypoints_actual);
    if(first_iteration)
    {
        detector.detect(previous_frame_l,keypoints_old);
        first_iteration = false;
    }

    //Matching characteristics
    extractor.compute(actual_frame_l,keypoints_actual,descriptors_actual);
    extractor.compute(previous_frame_l,keypoints_old,descriptors_old);
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

        angle_prov = ((atan2(x,y)*180) / PI)*(-1);


        //Computing the media

        if ( angle_prov > -85 && angle_prov <  85)
        {
            distance.push_back(sqrt(x*x+y*y));
            angle.push_back(angle_prov);

            media_distance = media_distance + sqrt(x*x+y*y);
            media_angle = media_angle + angle_prov;
            it_counter_actual.push_back(it->queryIdx);
            it_counter_old.push_back(it->trainIdx);
            contador ++;

        }
    }

    media_distance = media_distance/contador;
    media_angle = media_angle/contador;

    cout << "distancia: " << media_distance << endl;
    cout << "angulo: " << media_angle << endl;

    if (it_counter_actual.empty() || it_counter_old.empty())
    {
        cout << "fail" << endl;
        return;
    }

    good_keypoints_actual.clear();
    good_keypoints_old.clear();



    /*if distance[i] && angle[i] belong to the working area add the points to the new keypoint actual and new
    it is necessary to take in account that keypoints_old/new haven't got the same size*/

    for (int i =0; i < contador; i++)
    {
        if(distance.at(i) > 0.75 * media_distance && distance.at(i) < 1.25  * media_distance)
        {
            //            if(angle[i] > 0.85 * media_angle && angle[i] < 1.15 * media_angle)
            //            {
            good_keypoints_actual.push_back(keypoints_actual[it_counter_actual[i]]);
            good_keypoints_old.push_back(keypoints_old[it_counter_old[i]]);
            //            }
        }
    }

    keypoints_old.clear();
    keypoints_old = keypoints_actual;

    ///Test
    /*drawKeypoints(actual_frame_l, good_keypoints_actual, output_actual);
    drawKeypoints(previous_frame_l, good_keypoints_old, output_old);
    cout << "points actual: " << good_keypoints_actual.size() << endl;
    cout << "points previous: " << good_keypoints_old.size() << endl;
    imshow("actual", output_actual);
    imshow("old", output_old);
    waitKey(1);*/
}
