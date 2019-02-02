/**
 * 1: Two colored tags are mounted on the endowrist tools.
 *      First, get the positions of the two tages from the input point cloud based on the color.
 *      Then calculate the position of the end-effector based on the relative positions.
 * 2: Also get the position of the colored points of the object to be grasped
 * Yongming Qin
 * 2019/01/22
 * v1: based on pcl_filter_color_based_v1.cpp
 */

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/point_types_conversion.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/centroid.h> // for computer3Dcentroid()

#include <pc_analysis/obj_pos.h>

#include <eigen3/Eigen/Dense> // for Eigen::Vector3d

#include <string>
#include <iostream>
#include <vector>
using namespace std;

const int m2um = 1000000;
const vector<double> RED = {0, 12, 191, 226, 110, 155};
const vector<double> GREEEN = {93, 114, 71, 121, 0, 83};
const vector<double> YELLOW = {23, 56, 200, 256, 177, 256};
const vector<double> BLUE = {117, 145, 127, 211, 83, 144};
// 6 hsv filter threshold values for upper tag, lower tag, object
vector<double> th_upper(RED), th_lower(GREEEN), th_obj(YELLOW);

Eigen::Vector3d v_eef, v_obj; // result position of the eef and object

void callback_pos(const sensor_msgs::PointCloud2ConstPtr & pc_in);
int obj_pos_hsv(const sensor_msgs::PointCloud2ConstPtr &pc_in, vector<double> th,
                    Eigen::Vector3d &v_pos);

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pc_analysis");

    // from color_detection. The frist two values are multiplied by two in "build the condition"
    if (argc == 13) {
        for (int i = 1; i <= 6; ++i) {
            th_upper[i] = stod(argv[i]);
            th_lower[i] = stod(argv[i+6]);
        }
    } else if (argc == 1) { 
        // use default thresholds
    } else {
        cout << "Wrong arguments!" << endl;
        return -1;
    }

    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, callback_pos);
    // pulisher that publishes the result
    ros::Publisher pub_pos = nh.advertise<pc_analysis::obj_pos>("pos_VSC", 1);
    pc_analysis::obj_pos pos;

    // Subscribe for a period so that the value is updated.
    ros::Time t;
    t = t.now();
    ros::Rate loop_rate(10);
    while (ros::ok() && (t.now() - t).toSec() <= 3600)
    {
        for (int i=0; i < 3; ++i) {
            pos.pos[i] = int(v_eef[i] * m2um);
            pos.pos[i+3] = int(v_obj[i] * m2um);
        }
        pub_pos.publish(pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/** The callback function for the subscriber.
 */
void callback_pos(const sensor_msgs::PointCloud2ConstPtr & pc_in)
{
    Eigen::Vector3d v_upper, v_lower;
    int n_upper = obj_pos_hsv(pc_in, th_upper, v_upper);
    int n_lower = obj_pos_hsv(pc_in, th_lower, v_lower);
    int n_obj = obj_pos_hsv(pc_in, th_obj, v_obj);
    v_eef = 2*v_lower - v_upper;
    cout << "upper: " << n_upper << "\tlower: " << n_lower << "\tobj: " << n_obj << endl;
    cout << v_eef.transpose() << "\n" << v_obj.transpose() << endl; 
}

/** get the center of the colored points.
 */
int obj_pos_hsv(const sensor_msgs::PointCloud2ConstPtr &pc_in, vector<double> th,
                Eigen::Vector3d &v_pos)
{
    // from ros msg to pcl data type
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pc_in, *cloud_rgb_in);

    //std::vector<int> indices;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::removeNaNFromPointCloud(*cloud_rgb_ori, *cloud_rgb, indices);

    //cout << "\n\nSize of cloud_rgb_in: " << cloud_rgb_in->points.size() << endl;

    // from rgb to hsv
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_in(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_rgb_in, *cloud_hsv_in);
    //cout << "Size of cloud_hsv: " << cloud_hsv_in->points.size() << endl;

    // Tranmit the position. Not sure if this is necessary.
    for (size_t i = 0; i < cloud_hsv_in->points.size(); ++i)
    {
        cloud_hsv_in->points[i].x = cloud_rgb_in->points[i].x;
        cloud_hsv_in->points[i].y = cloud_rgb_in->points[i].y;
        cloud_hsv_in->points[i].z = cloud_rgb_in->points[i].z;
    }

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, th[0]*2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, th[1]*2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, th[2] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, th[3] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, th[4] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, th[5] / 255)));
        
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZHSV> cond_rem;
    cond_rem.setInputCloud(cloud_hsv_in);
    cond_rem.setCondition(range_cond);
    //cond_rem.setKeepOrganized(true); // if this exists, nothing is changed.

    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZHSV>);
    cond_rem.filter(*cloud_color);
    //cout << "Size of cloud_color: " << cloud_color->points.size() << endl;


    pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> out_rem;
    // build the filter
    out_rem.setInputCloud(cloud_color);
    out_rem.setRadiusSearch(0.02);
    out_rem.setMinNeighborsInRadius(5);
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color_radius(new pcl::PointCloud<pcl::PointXYZHSV>);
    out_rem.filter(*cloud_color_radius);
    //cout << "Size of cloud_color_radius: " << cloud_color_radius->points.size() << endl;

    //centroid obtaining    
    Eigen::Vector4d cent_cam, cent_world, cent_raven;
    int ret = pcl::compute3DCentroid(*cloud_color_radius, cent_cam);
    //cout << "center (camera frame): " << cent_cam.transpose() << endl;

    // transfom the position to world (moveit) frame
    // args="-0.27 0.095 0 -1.5708 0.61 0 0_link zed_left_camera_frame 100"
    const double theta = 0.8776; // rad asin(22/26)
    const double T_camera[3] = {-0.27, 0.11, 0.03}; // camera center of two lenses
    cent_world[0] = T_camera[0] + cent_cam[1];
    cent_world[1] = T_camera[1] - ( cent_cam[0]*cos(theta) + cent_cam[2]*sin(theta) );
    cent_world[2] = T_camera[2] - ( cent_cam[0]*sin(theta) - cent_cam[2]*cos(theta) );
    //cout << "center (world frame): " << cent_world.transpose() << endl;

    cent_raven[0] = cent_world[2]; cent_raven[1] = cent_world[1]; cent_raven[2] = -0.21 - cent_world[0];
    //cout << "center (raven frame): " << cent_raven.transpose() << endl;
    //cout << "center (raven frame m to um): " << cent_raven.transpose() * m2um << endl;
    
    for (int i=0; i<3; ++i) v_pos[i] = cent_raven[i];

    return cloud_color_radius->points.size();
}