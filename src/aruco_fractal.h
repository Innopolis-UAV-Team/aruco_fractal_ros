//
// Created by op on 11.10.2019.
//

#ifndef ARUCO_FRACTAL_ROS_ARUCO_FRACTAL_H
#define ARUCO_FRACTAL_ROS_ARUCO_FRACTAL_H

#include "cvdrawingutils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "aruco/fractaldetector.h"
#include "aruco/aruco_cvversioning.h"

//ROS Images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Camera calibration
#include <sensor_msgs/CameraInfo.h>

// Tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;
using namespace cv;

class ArucoFractalRos {

public:
    ArucoFractalRos(int argc,char **argv);
    ~ArucoFractalRos();

    void initRos();
    void rosRun();
    void readParameters();

//    Params
protected:
    float marker_size = 0.27;
    string parent_frame;
    string child_frame;
    string marker_frame = "fractal";

    string imageTopicName = "/image_raw";
    float rate = 15.;
    string camera_info_topic_name = "/camera_info";
    string markerTopicName = "aruco_marker";
    int window_size = 1;
    string marker_config = "FRACTAL_4L_6";
    string calibration_path = "";
    bool invert_image = false;
    bool display_image = true;
    bool get_image = false;
protected:      // camera info
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
    aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                         bool useRectifiedParameters);
    aruco::CameraParameters camera_parameters;
    bool camera_init_flag = false;
    bool readParametersFromPath(std::string filename);

protected:      // camera image
    cv_bridge::CvImagePtr cvImage;
    cv::Mat imageMat;
    cv_bridge::CvImage out_msg;

    //Subscriber
    image_transport::Subscriber imageSubs;
    image_transport::Publisher image_pub;

    image_transport::ImageTransport* imageTransport;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
protected:
    bool isGetFractalPosition(cv::Mat &img, geometry_msgs::Transform &transform);
    geometry_msgs::Transform marker_pose;
protected:
    ros::Publisher markerPub;

    // Lists of average
    std::vector<float> averageX;
    std::vector<float> averageY;
    std::vector<float> averageZ;

    float getAverageValue(vector<float> vector);
    geometry_msgs::PoseStamped markerPoseMsgs;
protected:
    geometry_msgs::Transform arucoMarker2Tf(aruco::FractalDetector fDetector);
    tf2_ros::TransformBroadcaster* tfTransformBroadcaster;
    tf2_ros::Buffer tfBuffer;
private:
    geometry_msgs::TransformStamped transform_stamped_;

};


struct   TimerAvrg{
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;
    TimerAvrg(int _n=30)
    {n=_n;
    times.reserve(n);
    }
    inline void start()
    {
        begin= std::chrono::high_resolution_clock::now();
    }
    inline
    void stop()
    {
        end= std::chrono::high_resolution_clock::now();
        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if ( times.size()<n)
            times.push_back(duration);
        else{
            times[curr]=duration;
            curr++;
            if(curr>=times.size())
                curr=0;
            }
    }
        double getAvrg(){double sum=0;
        for(auto t:times) sum+=t;
        return sum/double(times.size());
    }};

static TimerAvrg Tdetect, Tpose;

#endif //ARUCO_FRACTAL_ROS_ARUCO_FRACTAL_H
