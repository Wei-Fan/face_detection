#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Point.h"//added by Wade

#include "face_detection.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;

class image_node_c
{
public:
    image_transport::Publisher pub;
    image_transport::Subscriber sub;
    ros::Publisher faceloc_pub;//added by Wade

    geometry_msgs::Point faceloc;
    sensor_msgs::ImagePtr msg;
    cv::Mat result;
    cv::Mat tem;
    int fullwidth;
    int fullheight;
    cv::Rect cutBB;

    seeta::FaceDetection detector;
    image_node_c(ros::NodeHandle& nh, const char* configPath);
    ~image_node_c();
    void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg);
    void detectAndDraw(cv::Mat img);

};

image_node_c::image_node_c(ros::NodeHandle& nh, const char* configPath)
:detector(configPath)
,fullheight(0)
,fullwidth(0)
{
  image_transport::ImageTransport it(nh);
  faceloc_pub = nh.advertise<geometry_msgs::Point>("/face_loc", 1);//added by Wade
  pub = it.advertise("/face", 1);
  sub = it.subscribe("/ardrone/image_raw",1,&image_node_c::imageCallback,this);//modif
}

image_node_c::~image_node_c()
{}


void image_node_c::imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      /*转化成CVImage*/
       cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8);
       cout<<"transforming"<<endl;
       cv::waitKey(1);
    }

    catch (cv_bridge::Exception& e)
    {
      cout << "!!!" << endl;
    //ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
    }
    fullwidth = cv_ptr->image.cols;
    fullheight = cv_ptr->image.rows;

    tem = cv_ptr->image;

    detectAndDraw(tem);
    //cv::resize(tem, tem, cv::Size(40,40), (0,0), (0,0), cv::INTER_LINEAR);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
    pub.publish(msg);

}



void image_node_c::detectAndDraw(cv::Mat img)
{
  //seeta::FaceDetection detector("/home/rover/catkin_ws/src/fdetection/model/seeta_fd_frontal_v1.0.bin");
  detector.SetMinFaceSize(20);
  detector.SetScoreThresh(2.f);
  detector.SetImagePyramidScaleFactor(0.8f);
  detector.SetWindowStep(4, 4);

  cv::Mat img_gray;

  if (img.channels() != 1)
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
  else
    img_gray = img;

  seeta::ImageData img_data;
  img_data.data = img_gray.data;
  img_data.width = img_gray.cols;
  img_data.height = img_gray.rows;
  img_data.num_channels = 1;
  std::vector<seeta::FaceInfo> faces = detector.Detect(img_data);

  cv::Rect face_rect;
  int32_t num_face = static_cast<int32_t>(faces.size());

  if(num_face > 0)
  {
    face_rect.x = faces[0].bbox.x;
    face_rect.y = faces[0].bbox.y;
    face_rect.width = faces[0].bbox.width;
    face_rect.height = faces[0].bbox.height;

    cv::rectangle(img, face_rect, CV_RGB(0, 0, 255), 4, 8, 0);
    /*added by Wade*/
    faceloc.x = (face_rect.tl().x + face_rect.br().x) * 0.5;
    faceloc.y = (face_rect.tl().y + face_rect.br().y) * 0.5;
    faceloc.z = (face_rect.tl().x - face_rect.br().x)*(face_rect.tl().x - face_rect.br().x)+(face_rect.tl().y - face_rect.br().y)*(face_rect.tl().y - face_rect.br().y);
    faceloc.x -= img.cols * 0.5;
    faceloc.y -= img.rows * 0.5;
    faceloc.z = sqrt(faceloc.z);
    faceloc_pub.publish(faceloc);

    if (face_rect.x >= 0 && (face_rect.x + face_rect.width) <= img.cols )
    {
        if(face_rect.y <= 0)
          {
            result = tem(cv::Rect(face_rect.x,0,face_rect.width,face_rect.y + face_rect.height));
          }
        else result = tem(cv::Rect(face_rect.x,face_rect.y,face_rect.width,face_rect.height));
    }

    cv::imshow("C_OUT_WINDOW", img);
    cv::waitKey(1);
  }
  else
      cout<<"no faces"<<endl;

}


int main(int argc, char** argv)
{
    string configPath = argv[1];
    
    const char *configPathC = configPath.c_str();
    ros::init(argc, argv, "face_detection");
    ros::NodeHandle nc;
    image_node_c inc(nc,configPathC);


    ros::Rate loop_rate(10);

    while (nc.ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

        return 0;
}

