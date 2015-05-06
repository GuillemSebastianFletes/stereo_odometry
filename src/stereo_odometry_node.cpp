#include <stereo_odometry.hpp>

using namespace message_filters;
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;



void callback(const ImageConstPtr &original, const ImageConstPtr &mask)
{
    stereo_odometry stereo_odometry_calculus;

    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(original, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat original_image_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    original_image_=cv_disp_ptr->image;

    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(mask, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat image_mask_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    image_mask_=cv_disp_ptr->image;


    stereo_odometry_calculus.calculus(original_image_, image_mask_);
}

int main(int argc, char** argv)
{
    std::string original_image_name_;
    std::string image_mask_name_;

    ros::init(argc, argv, "stereo_odometry");
    cout<<"starting the program"<<endl;
    ros::NodeHandle nh;
    nh.getParam("/original_image_name",original_image_name_);
    nh.getParam("/image_mask_name",image_mask_name_);
    ros::Publisher translation_pub = nh.advertise<std_msgs::Float64>("translation", 1);
    ros::Publisher rotation_pub = nh.advertise<std_msgs::Float64>("rotation", 1);
    ros::Rate loop_rate(10);

    //sincronizer initialization
    message_filters::Subscriber<Image> original_sub(nh, original_image_name_.c_str(), 10);
    message_filters::Subscriber<Image> mask_sub(nh, image_mask_name_.c_str(), 10);
    TimeSynchronizer<Image, Image> sync(original_sub, mask_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
