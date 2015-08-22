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
    char format[] = ".png";
    char name[50];
    char name_old[50];
    int n;

    ///images initialitation
    Mat actual_frame;
    Mat previous_frame;


    stereo_odometry odometria_estereo;


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

        odometria_estereo.translation(actual_frame, previous_frame);
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"duration: "<< duration <<'\n';


    }

    return 0;
}
