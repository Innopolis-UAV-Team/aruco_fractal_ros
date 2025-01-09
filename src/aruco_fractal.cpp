//
// Created by op on 11.10.2019.
//

#include <numeric>
#include "aruco_fractal.h"

cv::Mat __resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}
////////// Drone Aruco Eye ///////////
ArucoFractalRos::ArucoFractalRos(int argc, char **argv)
{
    //Ros Init
    ros::init(argc, argv, "aruco_fractal_node");
    ros::NodeHandle nh;

    imageTransport = new image_transport::ImageTransport(nh);
    tfTransformBroadcaster=new tf2_ros::TransformBroadcaster();

    // Read parameters
    readParameters();
    // Init
    initRos();
    ros::spinOnce();
    // End
    return;
}

ArucoFractalRos::~ArucoFractalRos()
{
    // Delete
    if(imageTransport)
        delete imageTransport;
    if(tfTransformBroadcaster)
        delete tfTransformBroadcaster;

    return;
}

void ArucoFractalRos::initRos() {
    ros::NodeHandle nh;

    imageSubs = imageTransport->subscribe(imageTopicName, 1, &ArucoFractalRos::imageCallback, this);
    image_pub = imageTransport->advertise("/aruco/image", 1);
    markerPub = nh.advertise<geometry_msgs::PoseStamped>(markerTopicName, 10, true);

    if (calibration_path.empty())
    {
    cameraInfoSub=nh.subscribe(camera_info_topic_name, 1, &ArucoFractalRos::cameraInfoCallback, this);
    }
    else{
        if (readParametersFromPath(calibration_path)){
            cout << "calibration fom path init !" << endl;
        }
    }
}



bool ArucoFractalRos::readParametersFromPath(std::string filename)
{
    if(filename.empty())
    {
        cout<<"Camera calibration file is empty"<< camera_parameters<<std::endl;

        return false;
    }
    camera_parameters.readFromXMLFile(filename);
    if(camera_parameters.isValid())
    {
        camera_init_flag = true;
//        cout<<"Camera calibration parameters received!"<< camera_parameters<<std::endl;
        return true;
    }

    cout<<"Camera ERROR calibration NOT SET"<< camera_parameters<<std::endl;
    return false;
}


void ArucoFractalRos::readParameters()
{
    // Config files
    //
    ros::NodeHandle nh;

    ros::param::param<float>("~rate", rate, rate);
    std::cout<<"rate="<<rate<<std::endl;

    ros::param::param<float>("~marker_size", marker_size, marker_size);
    std::cout<<"marker_size="<<marker_size<<std::endl;


    ros::param::param<std::string>("~parent_frame", parent_frame, "map");
    std::cout<<"parent_frame="<<parent_frame<<std::endl;

    ros::param::param<std::string>("~marker_frame", marker_frame, marker_frame);
    std::cout<<"marker_frame="<<marker_frame<<std::endl;

    // Topic names//
    ros::param::param<std::string>("~image_topic_name", imageTopicName, imageTopicName);
    std::cout<<"image_topic_name="<<imageTopicName<<std::endl;
    //
    ros::param::param<std::string>("~camera_info_topic_name", camera_info_topic_name, camera_info_topic_name);
    std::cout<<"camera_info_topic_name="<<camera_info_topic_name<<std::endl;

    ros::param::param<std::string>("~marker_topic_name", markerTopicName, markerTopicName);
    std::cout<<"marker_topic_name="<<markerTopicName<<std::endl;

    ros::param::param<int>("~window_size", window_size, 1);
    std::cout<<"window_size="<<window_size<<std::endl;

    ros::param::param<std::string>("~marker_config", marker_config, marker_config);
    std::cout<<"marker_config="<<marker_config<<std::endl;

    ros::param::param<bool>("~display_image", display_image, display_image);
    std::cout<<"display_image="<<display_image<<std::endl;

    ros::param::param<bool>("~invert_image", invert_image, invert_image);
    std::cout<<"invert_image="<<invert_image<<std::endl;


    ros::param::param<std::string>("~calibration_path", calibration_path, calibration_path);
    std::cout<<"calibration_path="<<calibration_path<<std::endl;

    return;
}

void ArucoFractalRos::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    camera_parameters = rosCameraInfo2ArucoCamParams(msg, true);

    cameraInfoSub.shutdown();
    camera_init_flag = true;
    cout<<"[AE-ROS] Camera calibration parameters received!"<< camera_parameters<<std::endl;
    return;
}


aruco::CameraParameters ArucoFractalRos::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                  bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cv::Mat distorsionCoeff(4, 1, CV_32FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
        cameraMatrix.setTo(0);
        cameraMatrix.at<float>(0,0) = cam_info.P[0];   cameraMatrix.at<float>(0,1) = cam_info.P[1];   cameraMatrix.at<float>(0,2) = cam_info.P[2];
        cameraMatrix.at<float>(1,0) = cam_info.P[4];   cameraMatrix.at<float>(1,1) = cam_info.P[5];   cameraMatrix.at<float>(1,2) = cam_info.P[6];
        cameraMatrix.at<float>(2,0) = cam_info.P[8];   cameraMatrix.at<float>(2,1) = cam_info.P[9];   cameraMatrix.at<float>(2,2) = cam_info.P[10];

        for(int i=0; i<4; ++i)
            distorsionCoeff.at<float>(i, 0) = 0;
    }
    else
    {
        for(int i=0; i<9; ++i)
            cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_info.K[i];

        if(cam_info.D.size() == 4)
        {
            for(int i=0; i<4; ++i)
                distorsionCoeff.at<float>(i, 0) = cam_info.D[i];
        }
        else
        {
            ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
            for(int i=0; i<4; ++i)
                distorsionCoeff.at<float>(i, 0) = 0;
        }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}


void ArucoFractalRos::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (get_image)
    {
        ROS_INFO("image in processing. skip update");
        return;
    }

    child_frame = msg->header.frame_id;
    //Transform image message to Opencv to be processed
    try
    {
        cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
        imageMat=cvImage->image;
	if(invert_image)
	{
		cv::bitwise_not ( imageMat,  imageMat);
	}

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (!camera_init_flag)
    {
        cout << "Set camera params" << endl;
        return;
    }
    get_image = true;

    return;
}

float ArucoFractalRos::getAverageValue(vector<float> v)
{
    float average;
    if (v.size() == 1)
    {
        return v[0];
    }
    average = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    return average;
}


geometry_msgs::Transform ArucoFractalRos::arucoMarker2Tf(aruco::FractalDetector fDetector)
{
    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(fDetector.getRvec(), rot);
    cv::Mat tran = fDetector.getTvec();

    tf2::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                          rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                          rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    tf2::Vector3 tf_orig(tran.at<double>(0,0), tran.at<double>(1,0), tran.at<double>(2,0));

    tf2::Transform tf2_transform=tf2::Transform(tf_rot, tf_orig);

    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);


    return transform;
}

bool ArucoFractalRos::isGetFractalPosition(cv::Mat &img, geometry_msgs::Transform &transform)
{
    aruco::FractalDetector FDetector;
    FDetector.setConfiguration(marker_config);
    FDetector.setParams(camera_parameters, marker_size);
    // Ok, let's detect
    Tdetect.start();
    bool state = false;
    try {
        if (FDetector.detect(img)) {
//        cout << "\r\rTime detection: " << Tdetect.getAvrg()*1000 << " milliseconds"<<std::endl;
            Tdetect.stop();
            if (display_image)
                FDetector.drawMarkers(img);
            state = true;

            //Pose estimation
            Tpose.start();

            if (FDetector.poseEstimation()) {
                Tpose.stop();
                transform = arucoMarker2Tf(FDetector);
                state = true;
                vector<aruco::Marker> markers = FDetector.getMarkers();
                if (display_image)
                    FDetector.draw3d(img); //3d
            } else {
                if (display_image)
                    FDetector.draw2d(img);
            }
        }
    }
    catch(...){
        ROS_INFO("ERROR FDETECTOR");
    }
    return state;
}

void ArucoFractalRos::rosRun() {

        geometry_msgs::TransformStamped listener_transform_stamped;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate r(rate);
        ROS_INFO("Set rate: %f", rate);

        markerPoseMsgs.header.frame_id = parent_frame;

        // Clear of vectors
        averageX.clear();
        averageY.clear();
        averageZ.clear();

        while (ros::ok()) {

            // find pose in get image
            if (get_image)
            {
                if (isGetFractalPosition(imageMat, marker_pose)) {
                    transform_stamped_.header.stamp = ros::Time::now();
                    transform_stamped_.header.frame_id = child_frame;
                    // Child frame id
                    transform_stamped_.child_frame_id = marker_frame;
                    // Transform
                    transform_stamped_.transform = marker_pose;
                    // Publish
                    tfTransformBroadcaster->sendTransform(transform_stamped_);

                    // listener
                    try {
                        listener_transform_stamped = tfBuffer.lookupTransform(parent_frame,
                                                                              marker_frame,
                                                                              ros::Time(0.0));
                        if (window_size > 1)
                        {
                            //  with filter
                            averageX.push_back(listener_transform_stamped.transform.translation.x);
                            averageY.push_back(listener_transform_stamped.transform.translation.y);
                            averageZ.push_back(listener_transform_stamped.transform.translation.z);


                            if (averageX.size() > window_size) {
                                auto iter = averageX.cbegin(); // указатель на первый элемент
                                averageX.erase(iter);
                            }

                            if (averageY.size() > window_size) {
                                auto iter = averageY.cbegin(); // указатель на первый элемент
                                averageY.erase(iter);
                            }

                            if (averageZ.size() > window_size) {
                                auto iter = averageZ.cbegin(); // указатель на первый элемент
                                averageZ.erase(iter);
                            }

                            // publish to PoseStamped topic
                            markerPoseMsgs.pose.position.x = getAverageValue(averageX);
                            markerPoseMsgs.pose.position.y = getAverageValue(averageY);
                            markerPoseMsgs.pose.position.z = getAverageValue(averageZ);


                        }
                        else
                        {
                            //  without filter
                            markerPoseMsgs.pose.position.x = listener_transform_stamped.transform.translation.x;
                            markerPoseMsgs.pose.position.y = listener_transform_stamped.transform.translation.y;
                            markerPoseMsgs.pose.position.z = listener_transform_stamped.transform.translation.z;
                        }

                        markerPoseMsgs.header.stamp = ros::Time::now();

                        markerPoseMsgs.pose.orientation.x = listener_transform_stamped.transform.rotation.x;
                        markerPoseMsgs.pose.orientation.y = listener_transform_stamped.transform.rotation.y;
                        markerPoseMsgs.pose.orientation.z = listener_transform_stamped.transform.rotation.z;
                        markerPoseMsgs.pose.orientation.w = listener_transform_stamped.transform.rotation.w;

                        markerPub.publish(markerPoseMsgs);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s", ex.what());
                    }

                }

                if (display_image) {

                    out_msg.header.stamp = ros::Time::now();
                    out_msg.header.frame_id = child_frame;
                    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
                    out_msg.image = imageMat;

                    image_pub.publish(out_msg.toImageMsg());
                }
            get_image = false;

            }
            ros::spinOnce();
            r.sleep();
        }

    return;
}

