#include <stereo_odometry.hpp>

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
    char path1[] = "/home/guillem/07/image_1/";
    char format[] = ".png";
    char name_l[50];
    char name_old_l[50];
    char name_r[50];
    char name_old_r[50];
    int n;

    ///images initialitation
    Mat actual_frame_l;
    Mat previous_frame_l;

    Mat actual_frame_r;
    Mat previous_frame_r;


    stereo_odometry odometria_estereo;


    for (int i = 1; i < 1101; i++)
    {
        start = std::clock();
         ///image name initialitation

        //Izquierda
        n = sprintf(name_l,"%s%.6d%s", path, i, format);
        n = sprintf(name_old_l,"%s%.6d%s",path,i-1,format);
        // cout << name << "   " << name_old << endl;

        //Derecha
        n = sprintf(name_r,"%s%.6d%s", path1, i, format);
        n = sprintf(name_old_r,"%s%.6d%s",path1,i-1,format);
        // cout << name << "   " << name_old << endl;

        ///Reading the images
        actual_frame_l = imread(name_l, CV_8UC1);
        previous_frame_l = imread(name_old_l, CV_8UC1);

        actual_frame_r = imread(name_r, CV_8UC1);
        previous_frame_r = imread(name_old_r, CV_8UC1);

        odometria_estereo.translation(actual_frame_l, previous_frame_l, actual_frame_r, previous_frame_r);
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"duration: "<< duration <<'\n';


    }

    return 0;
}
