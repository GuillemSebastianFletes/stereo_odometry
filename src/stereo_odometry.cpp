#include <stereo_odometry.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816

// Constructor
stereo_odometry::stereo_odometry()
{
    //first execution variable
    first_execution_ = true;
}

// Destructor
stereo_odometry::~stereo_odometry(){}

// Init
void stereo_odometry::init(){}

void stereo_odometry::calculus(Mat &final_image, Mat &mask)
{
    //Publisher initialitation
    ros::NodeHandle n;
    ros::Publisher translation_pub = n.advertise<std_msgs::Float64>("translation", 1);
    ros::Publisher rotation_pub = n.advertise<std_msgs::Float64>("rotation", 1);

    //Image pre-procesing, to improve the features detector we are gong to heighten the borders
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    Mat grad;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    GaussianBlur( final_image, final_image, Size(3,3), 0, 0, BORDER_DEFAULT );//reduce gausian noise

    //border detection using Sobel
    Sobel( final_image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT ); // Gradient X
    Sobel( final_image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT ); // Gradient Y
    convertScaleAbs( grad_x, abs_grad_x );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, final_image );

    ///optical flow calculous
    //condition to see if is the first atemp, because to calculate the optical flow the old and the new features are needed
    if (first_execution_)
    {
        cout<<"primera ejecucion"<<endl;
        prev_image_ = final_image.clone();
        prev_mask_ = mask.clone();
        first_execution_=false;
    }

    else
    {
        stereo_odometry::translation_calculus(final_image,mask);
        translation_pub.publish(translation_);
        /*trajectory_mono::rotation_calculus(final_image);
            rotation_pub.publish(rotation_);*/

        prev_image_ = final_image.clone();
        prev_mask_ = mask.clone();
    }
}

void stereo_odometry::translation_calculus(Mat &final_image, Mat &mask)
{
    //good features to track variables
    vector<Point2f> NewFeatures, OldFeatures;
    int maxCorners = 1000;
    double qualityLevel = 0.01;
    double minDistance = 5;
    int blockSize = 3;
    bool useHarrisDetector = true;
    double k = 0.04;

    //optical flows variables
    std::vector<uchar> status;
    std::vector<float> err;

    //translation images variables
    Mat new_translation;
    Mat old_translation;
    Mat new_mask;
    Mat old_mask;

    //translatioan calculus variables
    double angle_translation;
    std::vector<double> translation;
    std::vector<double> repeated_value;
    std::vector<int> number_repetitons;
    double media;
    double max_value = 0;
    bool elemento_no_presente;
    double x;
    double y;

    ///ROI calculus of the images and the masks
    new_translation = final_image(Rect(0,1*final_image.rows/3,final_image.cols, 2*final_image.rows/3));//actual frame
    old_translation = prev_image_(Rect(0,1*final_image.rows/3,final_image.cols, 2*final_image.rows/3));//prev frame
    new_mask = mask(Rect(0,1*final_image.rows/3,final_image.cols, 2*final_image.rows/3));//actual frame
    old_mask = prev_mask_(Rect(0,1*final_image.rows/3,final_image.cols, 2*final_image.rows/3));//actual frame

    /// Features detection
    goodFeaturesToTrack( old_translation, OldFeatures, maxCorners, qualityLevel, minDistance, old_mask, blockSize, useHarrisDetector, k );//prev image
    goodFeaturesToTrack( new_translation, NewFeatures, maxCorners, qualityLevel, minDistance, new_mask, blockSize, useHarrisDetector, k );//actual image

    ///Optical Flow calculus
    cv::calcOpticalFlowPyrLK(
                old_translation, new_translation, // 2 consecutive images
                OldFeatures, // input point positions in first im
                NewFeatures, // output point positions in the 2nd
                status,    // tracking success
                err      // tracking error
                );

    ///Checking outlaiers
    /*It is know that the module vectors obtained from the optical flow should not be larger than 1/8 * rows of the image.
     * This vector is the result of the join of the same point between frames.
     * At the same time we asume that the angulus of this vector might be higher than 30 but less than 150*/
    for( size_t i=0; i < status.size(); i++ )
    {
        if(status[i]) //if there is optical flow get the vector of the desplazament
        {
            x=(NewFeatures[i].x)-(OldFeatures[i].x);
            y=(NewFeatures[i].y)-(OldFeatures[i].y);
            angle_translation = (atan2 (y,x) * 180.0 / PI)*(-1);//angle calculation in degrees


            /*Cheking if the angle belongs to the comfort area*/
            if(sqrt((x*x)+(y*y)) < final_image.rows/8 && angle_translation > 30 && angle_translation < 150 && (sqrt((x*x)+(y*y))) >1)
            {
                translation.push_back(sqrt((x*x)+(y*y)));

            }

        }
    }
    if (translation.size()<=0)//if the variable is empty (no translation detected)
    {
        return;
    }

    ///Calcule repited values
    /*First of all, repeated_value needs to be initializated to avoid problems with the conditional stamenter of repetion
     * so, the first value of the vector is going to be the first value of the vector that contains the translation values.
     * and number of repetitions of the first value is set to 0
     */

    repeated_value.push_back(translation[0]);
    number_repetitons.push_back(0);

    for ( size_t i=0; i<translation.size(); i++)
    {
        elemento_no_presente = true;
        for (size_t a=0; a<repeated_value.size();a++)
        {
            if (translation[i] == repeated_value[a])
            {
                number_repetitons[a]++;
                elemento_no_presente = false;
                break;
            }
        }

        if (elemento_no_presente)
        {
            repeated_value.push_back(translation[i]);
            number_repetitons.push_back(1);
        }
    }


    /// Get value that apears more
    // max_value contains the position where the value with the maximum number of repetitions is located
    for (size_t i= 0; i<number_repetitons.size(); i++ )
    {
        if (number_repetitons[i] =  number_repetitons[max_value])
        {
            max_value = i;
        }
    }

    //media variables initalitation
    translation_.data = 0;
    media = 0;
    for (size_t i= 0; i<number_repetitons.size(); i++ )
    {
        if (number_repetitons[i] >=  0.95*number_repetitons[max_value])
        {
            translation_.data = repeated_value[i] + translation_.data;
            media++;
        }
    }
    translation_.data = translation_.data / media;
}
