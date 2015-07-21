#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"

static const std::string OPENCV_WINDOW = "Image window";
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	
	public:
		ImageConverter()
			: it_(nh_){
				// Subscribe to input video feed and publish output video feed
				image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
				&ImageConverter::imageCb, this);
				image_pub_ = it_.advertise("/image_facetrackers/output_video", 1);

				cv::namedWindow(OPENCV_WINDOW);
			}
	
		~ImageConverter(){
			cv::destroyWindow(OPENCV_WINDOW);
		}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		std::vector<Rect> faces;
  		Mat frame_gray;

  		cvtColor( frame, frame_gray, CV_BGR2GRAY );
  		equalizeHist( frame_gray, frame_gray );

  		//-- Detect faces
  		face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  		for( size_t i = 0; i < faces.size(); i++ ){
  		 	Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    			ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

    			Mat faceROI = frame_gray( faces[i] );
    			std::vector<Rect> eyes;

    			//-- In each face, detect eyes
    			eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    			for( size_t j = 0; j < eyes.size(); j++ ){
       				Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
       				int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       				cv::circle(cv_ptr->image, cv::Point(center), radius, CV_RGB(255,0,0))
     			}
  		}
		// Update GUI Window
   		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      	 	cv::waitKey(3);
       
       		// Output modified video stream
       		image_pub_.publish(cv_ptr->toImageMsg());
