#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Labeling.h"
#include "geometry_msgs/Twist.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

std_msgs::Empty emp_msg;

int hMin = 50;
int hMax = 120;
int sMin = 50;
int sMax = 180;
int vMin = 50;
int vMax = 180;

void sortCorners(vector<Point2f>& corners, Point2f center) {
  vector<Point2f> top, bot;

  for (unsigned int i = 0; i < corners.size(); i++) {
    if (corners[i].y < center.y)
      top.push_back(corners[i]);
    else
      bot.push_back(corners[i]);
  }

  Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
  Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
  Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
  Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

  corners.clear();
  corners.push_back(tl);
  corners.push_back(tr);
  corners.push_back(br);
  corners.push_back(bl);
}

void videoCallback(const sensor_msgs::ImageConstPtr img)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

  Mat image = cv_ptr->image;
  resize(image, image, image.size(), 0.002, 0.002);
  Mat hsv;
  LabelingBS labeling;
  RegionInfoBS *ri;

  const int height = image.rows;
  const int width = image.cols;

  Mat result(height, width, CV_8UC1);
  short *dst = new short[width * height];

  result = Mat::zeros(height, width, CV_8UC1);
  cvtColor(image, hsv, CV_BGR2HSV);

  for(int y=0; y<height;y++){
    for(int x=0; x<width; x++){
      int index = hsv.step*y+(x*3);
      if(hsv.data[index] >= hMin && hsv.data[index] <= hMax
        && hsv.data[index+1] >= sMin && hsv.data[index+1] <= sMax
        && hsv.data[index+2] >= vMin && hsv.data[index+1] <= vMax
      ) {
        result.data[result.step*y+x] = 255;
      }
    }
  }
  labeling.Exec((uchar *)result.data, dst, width, height, true, 30);
  ri = labeling.GetResultRegionInfo( 0 );
  if (!ri) return;
  int ltop_x,ltop_y,rbottom_x,rbottom_y;
  ri->GetMin(ltop_x,ltop_y);
  ri->GetMax(rbottom_x, rbottom_y);
  rectangle(image, Point(ltop_x, ltop_y), Point(rbottom_x, rbottom_y), Scalar(255, 255, 255));

  imshow("A", image);
  imshow("B", result);
}

void CannyThreshold(int, void*) {}

int main(int argc, char** argv)
{
  ROS_INFO("detect teniss ball");
  ros::init(argc, argv,"detect_teniss_ball");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);

  ros::Subscriber vid_sub = node.subscribe("/ardrone/image_raw", 10, &videoCallback);

  namedWindow("A");
  namedWindow("B");

  createTrackbar("H Min Threshold:", "A", &hMin, 180, CannyThreshold);
  createTrackbar("H Max Threshold:", "A", &hMax, 180, CannyThreshold);
  createTrackbar("S Min Threshold:", "A", &sMin, 180, CannyThreshold);
  createTrackbar("S Max Threshold:", "A", &sMax, 180, CannyThreshold);
  createTrackbar("V Min Threshold:", "A", &vMin, 180, CannyThreshold);
  createTrackbar("V Max Threshold:", "A", &vMax, 180, CannyThreshold);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}

