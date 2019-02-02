/******
 * Visual Servo Control
 * 1. Get the position of the eef and the object from pc_analysis.
 * 2. Use a feedback loop to make the eef move to the object.
 * Yongming Qin
 * 2019/01/23 based on motion_path_publish_v5.cpp
 * 
 *
*/

#include <ros/ros.h>
#include "Raven_PathPlanner.h"
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <limits>

#include <pc_analysis/obj_pos.h>

using namespace std;
using namespace Eigen;

struct state
{
    double jpos[6]; // the first three joints of left and right arms
    int pos[6];     // um
    int pos_d[6];   // um
    float ori[18];
    float ori_d[18];
};
string s_eef;

struct Seg_change
{ // representation of the change of each segment

    Seg_change() {
        zero_tl.x = 0; zero_tl.y = 0; zero_tl.z = 0;
        id_ori.x = 0; id_ori.y = 0; id_ori.z = 0; id_ori.w = 1;
        tf.translation = zero_tl; tf.rotation = id_ori;
    }
    void reset()
    {
        b_tl = false;
        b_rot = false;
        tf.translation = zero_tl;
        tf.rotation = id_ori;
        grasp = 0;
        duration = INT_MAX;
    }
    void keep_tl_reset()
    { // tf.translation does not change
        b_tl = false;
        b_rot = false;
        tf.rotation = id_ori;
        grasp = 0;
        duration = INT_MAX;
    }

    void info() const {
        cout << "tf.translation: " << tf.translation.x << " " << tf.translation.y << " " << tf.translation.z << endl;
        cout << "tf.rotation: " << tf.rotation.x << " " << tf.rotation.y << " "
            << tf.rotation.z << " " << tf.rotation.w << endl;
        cout << "start translation: " << (b_tl ? "yes" : "no") << endl;
        cout << "start rotation: " << (b_rot ? "yes" : "no") << endl;
        cout << "grasp, duration: " << grasp << " " << duration << endl;
    }
    void trans(int x, int y, int z) {
        tf.translation.x += x;
        tf.translation.y += y;
        tf.translation.z += z;
    }
    void set(int x, int y, int z) {
        tf.translation.x = x;
        tf.translation.y = y;
        tf.translation.z = z;
    }
    void set(Eigen::Vector3i v) {
        tf.translation.x = v[0];
        tf.translation.y = v[1];
        tf.translation.z = v[2];
    }


    // use bool for translation and rotation as they are not all target or accumulation
    bool b_tl = false;           // true start translation
    bool b_rot = false;          // true start rotation.
    geometry_msgs::Transform tf; // translation: the target position; rotation: direct accumulation effect
    int grasp = 0;    // left: + close
    int duration = INT_MAX; // 0: no limits
    //
    geometry_msgs::Vector3 zero_tl; // no position increment
    //
    geometry_msgs::Quaternion id_ori; // no rotation increment

};

// The callback function for the subscriber
state gold_s;
void ravenstate_callback(const raven_state msg)
{
    for (int arm = 0; arm < 2; arm++)
    {
        for (int i = 0; i < 3; i++)
        { // there are 8x2 elements. The fourth one is not used.
            // From /raven_state of r2_control, the first three joints are of degrees.
            // !! It is wrong for the thrid joint which is prismatic.
            gold_s.jpos[i + arm * 3] = msg.jpos[i + arm * 8] * 3.14159 / 180;
        }
        //gold_s.jpos[2+arm*3] -= 0.45; // insertion prismatic joint. based on the relation between urdf and r2_control

        // for future
        for (int i = 0; i < 9; i++)
        {
            gold_s.ori[i + arm * 9] = msg.ori[i + arm * 9];
            gold_s.ori_d[i + arm * 9] = msg.ori_d[i + arm * 9];
        }
    }

    //cout << "jpos:"; for (int i=0; i<3; ++i) cout << " " << gold_s.jpos[0]; cout << endl;

    for (int i = 0; i < 3; ++i)
    {
        gold_s.pos[i] = msg.pos[i];
        gold_s.pos_d[i] = msg.pos_d[i];
    }
}

Eigen::Vector3i msg_pos_obj, msg_pos_eff;
void obj_pos_callback(const pc_analysis::obj_pos msg)
{
    for (int i = 0; i < 3; ++i) {
        msg_pos_eef[i] = msg.pos[i];
        msg_pos_obj[i] = msg.pos[i+3];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_servo_control");
    int arm_flag = 0; // left arm

    ros::NodeHandle n;
    /*-------------------Subscribe raven_state topic. Get the starting position--------------*/
    ros::Subscriber sub_ravenstate = n.subscribe("ravenstate", 10, ravenstate_callback);
    ros::Rate loop_rate_sub(100); // subscrbe to get the actual position

    /*-------------------Subscribe------------*/
    ros::Subscriber sub_obj_pos = n.subscribe("obj_pos", 10, obj_pos_callback);

    /*-----------------------Publish the planned path to raven_automove topic--------------*/
    ros::Publisher pub_motion = n.advertise<raven_automove>("raven_automove", 1);
    ros::Rate loop_rate_pub(1000); // for below publisher
    raven_automove automove_msg;

    Seg_change change;
    // 0, 1 for left, right
    automove_msg.tf_incr[0].translation = change.zero_tl;
    automove_msg.tf_incr[0].rotation = change.id_ori;
    automove_msg.tf_incr[1].translation = change.zero_tl;
    automove_msg.tf_incr[1].rotation = change.id_ori;

    geometry_msgs::Vector3 delta_pos; //

    /*-------------------approach the target position--------------------------------*/
    // untill the actural eef is within a range of the desired position
    vector<Seg_change> path;

    if (true) // for future
    {
        // Subscribe for a period so that the value is updated. Then get the current raven state.
        ros::Time t, t_eef;
        t = t.now();
        while (ros::ok() && (t.now() - t).toSec() <= 1)
        {
            ros::spinOnce();
            loop_rate_sub.sleep();
        }
        Eigen::Vector3i pos_obj = msg_pos_obj;

        if (argc == 1) {
            // Approaching the object.
            change.set(pos_obj + Vector3i(50000, 20000, -10000));
            change.b_tl = true;
            path.push_back(change);
            // move a little down
            change.tf.translation.x -= 10000;
            path.push_back(change);
            // Close the Grasper. raven_tf not change
            change.keep_tl_reset();
            change.grasp = 1; // 1-5
            change.duration = 1;
            path.push_back(change);
            // move a little up
            change.keep_tl_reset();
            change.tf.translation.x += 40000; // 3cm
            change.b_tl = true;
            path.push_back(change);
            // move to the above of the container
            change.reset();
            change.set(-126241, -14389, -23611);
            change.b_tl = true;
            path.push_back(change);
            // open the grasper
            change.keep_tl_reset();
            change.grasp = -1;
            change.duration = 3;
            path.push_back(change);
            // move a little up
            change.keep_tl_reset();
            change.tf.translation.x += 30000; // 3cm
            change.b_tl = true;
            path.push_back(change);
            // go back to homing position
            change.reset();
            change.set(-77830, -24349, 13885);
            change.b_tl = true;
            path.push_back(change);

            cout << "The number of segments(8): " << path.size() << endl;
        }
        else if (stoi(argv[1]) == 0) {
            // Approaching the given target
            change.tf.translation.x = -77830;
            change.tf.translation.y = -20349;
            change.tf.translation.z = 13885;
            change.b_tl = true;
            path.push_back(change);
            cout << "The number of segments(1): " << path.size() << endl;
        }
        

        for (auto p : path) { p.info(); cout << endl; }

        string in_s;
        cout << "Start this segment? y?" << endl;
        while (cin >> in_s)
        {
            if (in_s == "y") {break;}
            else if (in_s == "e") {return 0;}
            else {cout << "Please type \"y\" if you want to start ." << endl;}
        }


        const int RANGE = 500;
        const int V = 3;

        int n_seg = 0;
        for (auto it = path.cbegin(); it != path.cend(); ++it)
        {
            if (ros::ok()) {
                cout << "The segment: " << n_seg << endl;
                n_seg++;
                it->info();
                cout << endl;
                ros::Duration(2.0).sleep();

                Eigen::Vector3i act_pos_d;
                //for (int i = 0; i < 3; ++i) act_pos_d[i] = gold_s.pos_d[i];
                for (int i = 0; i < 3; ++i) act_pos_d[i] = msg_pos_eff[i];
                cout << "actual eef position: " << act_pos_d[0] << " " << act_pos_d[1] << " " << act_pos_d[2] << endl;

                t = t.now();
                t_eff = t_eff.now();
                while ( ros::ok() &&
                ( (t.now() - t).toSec() <= it->duration )
                )
                {
                    // reset to zero in case error.
                    for (int i=0; i<2; ++i) {
                        automove_msg.tf_incr[i].translation = change.zero_tl;
                        automove_msg.tf_incr[i].rotation = change.id_ori;
                        automove_msg.del_pos[i] = 0;
                    }

                    // translation
                    if (it->b_tl) {
                        // every step, move 1um delta_pos (from experiment)
                        int incr[3], dif[3];

                        // In the experiment, data1.xd vibrates between two numbers.
                        //   So add this to make sure when 3 values are in range stop this loop.
                        int n_in_range = 0;

                        if ( (t_eff.now() - t_eff).toSec() > 2) {
                            for (int i = 0; i < 3; ++i) act_pos_d[i] = msg_pos_eff[i]; // get the new eff position
                            t_eff = t_eff.now();
                        }

                        dif[0] = it->tf.translation.x - act_pos_d[0];
                        dif[1] = it->tf.translation.y - act_pos_d[1];
                        dif[2] = it->tf.translation.z - act_pos_d[2];

                        // approaching the target represented by translation in 3 directions till very close
                        for (int i = 0; i < 3; ++i)
                        {
                            if (abs(dif[i]) < RANGE) {
                                incr[i] = dif[i];
                                n_in_range++;
                            }
                            else {incr[i] = (dif[i] > 0) ? V : -V;}
                            act_pos_d[i] += incr[i];
                        }
                        delta_pos.x = incr[0]; delta_pos.y = incr[1]; delta_pos.z = incr[2];
                        // translation
                        automove_msg.tf_incr[arm_flag].translation = delta_pos; // arm_flag, 0/1 is for left/right arm
                        
                        if (n_in_range == 3) {
                            break; // ensure going to next segment.
                        }
                    }
                    
                    // grasper
                    if (it->grasp != 0) {
                        automove_msg.del_pos[arm_flag] = it->grasp;
                    }

                    // rotation
                    if (it->b_rot) {
                        automove_msg.tf_incr[arm_flag].rotation = it->tf.rotation;
                    }

                    automove_msg.hdr.stamp = automove_msg.hdr.stamp.now();
                    
                    pub_motion.publish(automove_msg);
                    ros::spinOnce();
                    loop_rate_pub.sleep();
                }
            }
        }
        ros::Duration(1.0).sleep(); // wait till the arm stops movement.
    }

    return 0;
}
