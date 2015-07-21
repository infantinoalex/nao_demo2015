#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <pthread.h>

typedef struct {
    double rating;
    cv::Rect rect;
} ConsistentRect;

class FaceDetector 
{
private:
    ros::NodeHandle n;
    ros::Subscriber raw_image;
    
    //For initial face detection 
    cv::CascadeClassifier face_cascade;
    cv::RNG rng;

    int no_face_count;
    std::vector<ConsistentRect> consistent_rects;

public:
    FaceDetector();
    void begin_detection();
    void head_camera_processing(const sensor_msgs::Image::ConstPtr&);
    void detectAndDisplay(cv::Mat); 
    bool properColor(cv::Mat);
    void addConsistent(cv::Rect);
    bool isOverlapping(cv::Rect, cv::Rect);
    void decrementConsistentRects();
    int findBestIndex(cv::Mat);
    void tickFaceCount(int, int, cv::Mat);
    std::vector<cv::Rect> findConfirmedFaces(std::vector<cv::Rect>, cv::Mat);
};
