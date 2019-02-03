/**
 * Getting the projection image of the bead.
 * Yongming qin
 * 2019/02/02: convert the python version to C++ for future compatibility
 * 
 */

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace std;

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";

cv_bridge::CvImagePtr cv_ptr;
void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bead_projection");
    ros::NodeHandle nh;
    ros::ServiceClient client_set = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state;
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "bead";
    model_state.reference_frame = "world";
    geometry_msgs::Pose pose_set;

    ros::ServiceClient client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name = "bead";
    geometry_msgs::Pose pose_get;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber left_sub = it.subscribe("/vritual_zedm/left/image_raw", 1, imageCallback);
    //image_transport::Subscriber right_sub = it.subscribe("/vritual_zedm/right/image_raw", 1, rightCallback);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::destroyWindow(WINDOW);

    ros::Time t;
    float resolution = 0.05;
    for (float x = 0.1; x <= 0.4; x += resolution)
    {
        for (float y = -0.15; y <= 0.15; y += resolution)
        {
            for (float z = -0.15; z <= 0.15; z += resolution)
            {

                if (client_get.call(get_model_state))
                {
                    pose_get = get_model_state.response.pose;
                    cout << pose_get << endl;
                }
                else cout << "error" << endl;

                // ------------------------------------- //
                pose_set.position.x = x;
                pose_set.position.y = y;
                pose_set.position.z = z;
                pose_set.orientation.x = 0;
                pose_set.orientation.y = 0;
                pose_set.orientation.z = 0;
                pose_set.orientation.w = 1;
                model_state.pose = pose_set;

                //cout << model_state << endl;

                set_model_state.request.model_state = model_state;

                if (client_set.call(set_model_state))
                {
                    ROS_INFO("set model state success");
                    t = ros::Time::now();
                }
                else
                {
                    ROS_ERROR("Failed to call service");
                    return 1;
                }

                while ((ros::Time::now() - t).toSec() < 1)
                    ; // do nothing
                ros::spinOnce();
                stringstream name;
                name << x << "_" << y << "_" << z << ".jpg"; 
                cv::imwrite(name.str(), cv_ptr->image);
            }
        }
    } 
    return 0;
}
