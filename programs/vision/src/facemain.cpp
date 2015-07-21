#include "facedetect.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FaceDetect");
    FaceDetector fd;
    fd.begin_detection();   
 
    return 0;
}
