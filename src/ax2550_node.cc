#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ax2550/StampedEncoders.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <cmath>

#include "ax2550/ax2550.h"

using namespace ax2550;
using std::string;

AX2550 *mc;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
tf::TransformBroadcaster *odom_broadcaster;

static double ENCODER_RESOLUTION = 250*4;
static double METERS_PER_TICK = 0.00020991;
static double VELOCITY_MULTIPLIER = 0.008448;
double wheel_circumference = 0.0;
double wheel_base_length = 0.0;
double wheel_diameter = 0.0;
double encoder_poll_rate;
std::string odom_frame_id;
size_t error_count;
double channel_1_value = 0.0;
double channel_2_value = 0.0;

double rot_cov = 0.0;
double pos_cov = 0.0;

static double A_MAX = 20.0;
static double B_MAX = 20.0;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;

double wrapToPi(double angle) {
    angle += M_PI;
    bool is_neg = (angle < 0);
    angle = fmod(angle, (2.0*M_PI));
    if (is_neg) {
        angle += (2.0*M_PI);
    }
    angle -= M_PI;
    return angle;
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mc == NULL || !mc->isConnected())
        return;

    // Convert twist to wheel velocities for differential drive
    double right_wheel_vel = msg->linear.x + msg->angular.z * wheel_base_length/2;
    double left_wheel_vel = msg->linear.x - msg->angular.z * wheel_base_length/2;

    // Convert wheel velocities from meters per second to relative velocity for roboteq
    double right_relative = (right_wheel_vel/METERS_PER_TICK) * VELOCITY_MULTIPLIER;
    double left_relative = (left_wheel_vel/METERS_PER_TICK) * VELOCITY_MULTIPLIER;

    channel_1_value = left_relative;
    channel_2_value = right_relative;
}

void controlLoop() {
    // ROS_INFO("Relative move commands: %f %f", channel_1_value, channel_2_value);
    try {
    	mc->move(channel_1_value, channel_2_value);
    } catch(const std::exception &e) {
    	if (string(e.what()).find("did not receive") != string::npos
         || string(e.what()).find("failed to receive an echo") != string::npos) {
            ROS_WARN("Error commanding the motors: %s", e.what());
    	} else {
            ROS_ERROR("Error commanding the motors: %s", e.what());
            mc->disconnect();
    	}
    }
}

void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void warnMsgCallback(const std::string &msg) {
    ROS_WARN("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void debugMsgCallback(const std::string &msg) {
    ROS_DEBUG("%s", msg.c_str());
}

void queryEncoders() {
    // Make sure we are connected
    if(!ros::ok() || mc == NULL || !mc->isConnected())
        return;

    long encoder1, encoder2;
    ros::Time now = ros::Time::now();
    // Retreive the data
    try {
        mc->queryEncoders(encoder1, encoder2, true);
        if (error_count > 0) {
            error_count -= 1;
        }
    } catch(std::exception &e) {
        if (string(e.what()).find("failed to receive ") != string::npos
         && error_count != 10) {
            error_count += 1;
            ROS_WARN("Error reading the Encoders: %s", e.what());
        } else {
            ROS_ERROR("Error reading the Encoders: %s", e.what());
            mc->disconnect();
        }
        return;
    }

    double delta_time = (now - prev_time).toSec();
    prev_time = now;

    // Convert to mps for each wheel from delta encoder ticks
    double deltaD = (encoder1 + (encoder2))/2 * METERS_PER_TICK;
    double deltaTheta = ((encoder1 - (encoder2)) / wheel_base_length) * METERS_PER_TICK;
    double v = deltaD/delta_time;
    double w = deltaTheta/delta_time;
    prev_w = wrapToPi(prev_w + deltaTheta);
    prev_x = prev_x + deltaD * cos(prev_w);
    prev_y = prev_y + deltaD * sin(prev_w);

    ax2550::StampedEncoders encoder_msg;

    encoder_msg.header.stamp = now;
    encoder_msg.header.frame_id = "base_link";
    encoder_msg.encoders.time_delta = delta_time;
    encoder_msg.encoders.left_wheel = encoder1;
    encoder_msg.encoders.right_wheel = -encoder2;

    encoder_pub.publish(encoder_msg);

    // ROS_INFO("%f", prev_w);

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(prev_w);

    // Populate the msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = prev_x;
    odom_msg.pose.pose.position.y = prev_y;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = pos_cov;
    odom_msg.pose.covariance[7] = pos_cov;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = rot_cov;

    // odom_msg.twist.twist.linear.x = v/delta_time;
    odom_msg.twist.twist.linear.x = v;
    // odom_msg.twist.twist.angular.z = w/delta_time;
    odom_msg.twist.twist.angular.z = w;

    odom_pub.publish(odom_msg);

    // TODO: Add TF broadcaster
    // geometry_msgs::TransformStamped odom_trans;
    //     odom_trans.header.stamp = now;
    //     odom_trans.header.frame_id = "odom";
    //     odom_trans.child_frame_id = "base_footprint";
    //
    //     odom_trans.transform.translation.x = prev_x;
    //     odom_trans.transform.translation.y = prev_y;
    //     odom_trans.transform.translation.z = 0.0;
    //     odom_trans.transform.rotation = quat;
    //
    //     odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "ax2550_node");
    ros::NodeHandle n;
    prev_time = ros::Time::now();

    // Serial port parameter
    std::string port;
    n.param("serial_port", port, std::string("/dev/motor_controller"));

    // Wheel diameter parameter
    n.param("wheel_diameter", wheel_diameter, 0.3048);

    wheel_circumference = wheel_diameter * M_PI;

    // Wheel base length
    n.param("wheel_base_length", wheel_base_length, 0.9144);

    // Odom Frame id parameter
    n.param("odom_frame_id", odom_frame_id, std::string("odom"));

    // Load up some covariances from parameters
    n.param("rotation_covariance",rot_cov, 1.0);
    n.param("position_covariance",pos_cov, 1.0);

    // Setup Encoder polling
    n.param("encoder_poll_rate", encoder_poll_rate, 25.0);
    ros::Rate encoder_rate(encoder_poll_rate);

    // Odometry Publisher
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);

    // Encoder Publisher
    encoder_pub = n.advertise<ax2550::StampedEncoders>("encoders", 5);

    // TF Broadcaster
    odom_broadcaster = new tf::TransformBroadcaster;

    // cmd_vel Subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);

    // Spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()) {
        ROS_INFO("AX2550 connecting to port %s", port.c_str());
        try {
            mc = new AX2550();
            mc->warn = warnMsgCallback;
            mc->info = infoMsgCallback;
            mc->debug = debugMsgCallback;
            mc->connect(port);
        } catch(std::exception &e) {
            ROS_ERROR("Failed to connect to the AX2550: %s", e.what());
            if (mc != NULL) {
            	mc->disconnect();
            }
        }
        int count = 0;
        while(mc != NULL && mc->isConnected() && ros::ok()) {
            queryEncoders();
            if (count == 1) {
                controlLoop();
                count = 0;
            } else {
                count += 1;
            }
			encoder_rate.sleep();
        }
        if (mc != NULL) {
        	delete mc;
        }
        mc = NULL;
        if(!ros::ok())
            break;
        ROS_INFO("Will try to reconnect to the AX2550 in 5 seconds.");
        for (int i = 0; i < 100; ++i) {
        	ros::Duration(5.0/100.0).sleep();
        	if (!ros::ok())
        		break;
        }
        channel_1_value = 0.0;
        channel_2_value = 0.0;
    }

    spinner.stop();

    return 0;
}
