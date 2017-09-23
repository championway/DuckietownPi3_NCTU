#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

using namespace cv;
using namespace std;

void callback(const sensor_msgs::ImageConstPtr& imageMap)
{
    ROS_INFO("Received image %dx%d", imageMap->width, imageMap->height);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(imageMap, imageMap->encoding); 

    cv::Mat im_pi = cv::Mat::zeros(480, 640, CV_8UC3);
    im_pi = cv_ptr->image.clone();

}

void callback_compressed(const sensor_msgs::CompressedImageConstPtr& imageMap)
{
    ROS_INFO("Received Compressed Image ");

    cv::Mat im_pi = cv::Mat::zeros(480, 640, CV_8UC3);
    im_pi = cv::imdecode(cv::Mat(imageMap->data), CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(im_pi, im_pi, CV_RGB2BGR);

    //YOU CODE HERE
    double fx = 507.8727;
    double fy = 507.8727;
    double px = 640.5;
    double py = 480.5;

    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;

    double x[63] = {-0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,-0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,-0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,
        -0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,-0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,-0.24,-0.18,-0.12,-0.06,0,0.06,0.12,0.18,0.24,-0.24,-0.18,
        -0.12,-0.06,0,0.06,0.12,0.18,0.24};
    double y[63] = {0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.37,0.37,0.37,0.37,0.37,0.37,0.37,0.37,0.37,0.43,
        0.43,0.43,0.43,0.43,0.43,0.43,0.43,0.43,0.49,0.49,0.49,0.49,0.49,0.49,0.49,0.49,0.49,0.55,0.55,0.55,0.55,0.55,0.55,0.55,0.55,0.55,0.61,0.61,0.61,0.61,
        0.61,0.61,0.61,0.61,0.61};
    //double z[63] = {-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,
    //    -0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,
    //   -0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09,-0.09};

    int a[63] = {77,212,347,476,601,723,845,960,1077,184,292,399,503,603,703,800,894,990,257,346,433,521,605,688,770,849,929,310,385,460,533,606,677,747,817,
        885,348,414,479,543,606,669,730,791,851,378,434,494,548,607,666,716,775,826,403,454,507,555,608,658,707,756,805};

    int b[63] = {687,686,683,679,677,674,670,667,665,646,642,641,639,638,636,634,634,630,616,615,614,613,612,610,609,608,607,597,596,595,594,593,592,591,590,
        589,581,581,580,579,579,577,577,577,576,571,570,570,570,568,568,568,567,567,560,560,559,559,557,559,558,558,556};

    for (int i = 0;i < 63;i++)
        {
            objPts.push_back(cv::Point3f(x[i],y[i],0));
            imgPts.push_back(cv::Point2f(a[i],b[i]));
        }
    printf("objPts");
    printf("imgPts");

    //TODO: need to add a few pairs of objPts and imgPts

    cv::Mat rvec, tvec;
    cv::Matx33f cameraMatrix(
            fx, 0, px,
            0, fy, py,
            0,  0,  1);
    cv::Vec4f distParam(0,0,0,0); // all 0?
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d wRo;
    wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);


    Eigen::Matrix<double, 3, 4> P;  // projection matrix
    Eigen::Matrix4d T;              // transformation
    T.topLeftCorner(3,3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;

    Eigen::Matrix<double, 3, 4> K;
    K <<
           fx,  0,  px, 0,
            0, fy,  py, 0,
            0,  0,   1, 0;
    P = K * T;
    std::cout << P << std::endl;
    std::cout << T << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_cpp_template_node");
    ros::NodeHandle nh("~");

    // read param from launch file
    std::string veh;

    // There are two ways to setup the param
    //nh.param<std::string>("veh", veh, "trabant"); // make sure you init nh by "~"
    ros::param::param<std::string>("~veh", veh, "jack");
    std::stringstream ss;
    ss << "/" << veh << "/camera_node/image/raw/compressed";
    ros::Subscriber sub = nh.subscribe(ss.str().c_str(), 1000, callback_compressed);
    ROS_INFO("Start opencv_cpp_template_node, and subscribe topic: %s", ss.str().c_str());
    ros::spin();
    return 0;
}
