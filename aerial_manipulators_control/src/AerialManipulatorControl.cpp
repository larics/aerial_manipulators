#include "ros/ros.h"
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

class AerialManipulatorControl {
    private:
        int rate_;
        bool simulation_flag_, uav_pose_meas_received_, arm_pose_meas_received_;

        geometry_msgs::PoseStamped uav_pose_meas_, arm_pose_meas_;

        Eigen::Matrix4d Tuav_arm_, Tworld_uav_origin_, Tuav_origin_world_;
        Eigen::Matrix4d Tend_effector_arm_, Tarm_end_effector_;

        rosgraph_msgs::Clock clock_;
    public:
        AerialManipulatorControl(int rate, bool simulation):
            rate_(rate),
            simulation_flag_(simulation),
            uav_pose_meas_received_(false),
            arm_pose_meas_received_(false) {

            Tuav_arm_ <<  1,  0,  0,  0,
                          0,  1,  0,  0,
                          0,  0,  1,  0,
                          0,  0,  0,  1;

            Tworld_uav_origin_ << 1, 0, 0, 0,
                                  0, 1, 0, 0, 
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;

            Tuav_origin_world_ << 1, 0, 0, 0,
                                  0, 1, 0, 0, 
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;

        };

        void uavPoseMeasOdomCb(const nav_msgs::Odometry &msg) {
            float position[3], q[4], orientationEuler[3];

            uav_pose_meas_received_ = true;
            uav_pose_meas_.header = msg.header;
            uav_pose_meas_.pose = msg.pose.pose;

            position[0] = msg.pose.pose.position.x;
            position[1] = msg.pose.pose.position.y;
            position[2] = msg.pose.pose.position.z;

            q[0] = msg.pose.pose.orientation.w;
            q[1] = msg.pose.pose.orientation.x;
            q[2] = msg.pose.pose.orientation.y;
            q[3] = msg.pose.pose.orientation.z;

            quaternion2euler(q, orientationEuler);

            getRotationTranslationMatrix(Tworld_uav_origin_, orientationEuler, position);
            Tuav_origin_world_ = Tworld_uav_origin_.inverse();
        };

        void armPoseMeasOdomCb(const geometry_msgs::PoseStamped &msg) {
            float position[3], q[4], orientationEuler[3];

            arm_pose_meas_received_ = true;
            arm_pose_meas_ = msg;

            position[0] = msg.pose.position.x;
            position[1] = msg.pose.position.y;
            position[2] = msg.pose.position.z;

            q[0] = msg.pose.orientation.w;
            q[1] = msg.pose.orientation.x;
            q[2] = msg.pose.orientation.y;
            q[3] = msg.pose.orientation.z;

            quaternion2euler(q, orientationEuler);

            getRotationTranslationMatrix(Tarm_end_effector_, orientationEuler, position);
            Tend_effector_arm_ = Tarm_end_effector_.inverse();
        };

        void LoadParameters(std::string file) {
            YAML::Node config = YAML::LoadFile(file);
            std::vector<double> manipulator_origin;

            manipulator_origin = config["origin"]["manipulator"].as<std::vector<double> >();

            Tuav_arm_ <<  cos(manipulator_origin[5])*cos(manipulator_origin[4]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])-sin(manipulator_origin[5])*cos(manipulator_origin[3]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])+sin(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[0],
                          sin(manipulator_origin[5])*cos(manipulator_origin[4]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])+cos(manipulator_origin[5])*cos(manipulator_origin[3]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])-cos(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[1],
                         -sin(manipulator_origin[4]),                             cos(manipulator_origin[4])*sin(manipulator_origin[3]),                                                                                    cos(manipulator_origin[4])*cos(manipulator_origin[3]),                                                                                  manipulator_origin[2],
                          0,                                                      0,                                                                                                                                        0,                                                                                                                                      1;
        };

        void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, float *orientationEuler, float *position) {
            float r11, r12, r13, t1, r21, r22, r23, t2;
            float r31, r32, r33, t3;

            float x, y, z;

            x = orientationEuler[0];
            y = orientationEuler[1];
            z = orientationEuler[2];

            r11 = cos(y)*cos(z);

            r12 = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);

            r13 = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

            r21 = cos(y)*sin(z);

            r22 = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);

            r23 = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

            r31 = -sin(y);

            r32 = cos(y)*sin(x);

            r33 = cos(x)*cos(y);

            t1 = position[0];
            t2 = position[1];
            t3 = position[2];


            rotationTranslationMatrix << 
                r11, r12, r13, t1,
                r21, r22, r23, t2,
                r31, r32, r33, t3,
                0,   0,   0,   1;
        };

        void quaternion2euler(float *quaternion, float *euler) {
            euler[0] = atan2(2 * (quaternion[0] * quaternion[1] + 
                quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1]
                + quaternion[2] * quaternion[2]));

            euler[1] = asin(2 * (quaternion[0] * quaternion[2] -
                quaternion[3] * quaternion[1]));

            euler[2] = atan2(2 * (quaternion[0]*quaternion[3] +
                quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] +
                quaternion[3] * quaternion[3]));
        };

        void getEndEffectorTransformationMatrix(Eigen::Matrix4d &transformationMatrix) {
            transformationMatrix = Tworld_uav_origin_*Tuav_arm_*Tarm_end_effector_;
        };

        bool isReady(void) {
            return arm_pose_meas_received_ && uav_pose_meas_received_;
        };

        void clockCb(const rosgraph_msgs::Clock &msg) {
            clock_ = msg;
        };

        ros::Time getTime() {
            ros::Time time;

            if (simulation_flag_)
                time = clock_.clock;
            else 
                time = ros::Time::now();

            return time;
        };
};


int main(int argc, char **argv)
{
    int rate;
    bool simulation_flag;
    double time, time_old, dt;
    Eigen::Matrix4d Tworld_end_effector;
    std_msgs::Float64MultiArray transformation_msg;

    transformation_msg.data.resize(16);

    std::string path = ros::package::getPath("aerial_manipulators_control");
    std::string parameters_file;

    ros::init(argc, argv, "aerial_manipulator_control");
    ros::NodeHandle n, private_node_handle_("~");

    private_node_handle_.param("rate", rate, int(100));
    private_node_handle_.param("simulation", simulation_flag, bool(true));
    private_node_handle_.param("parameters_file", parameters_file, std::string("/config/aerial_manipulator_parameters.yaml"));

    ros::Rate loop_rate(rate);

    AerialManipulatorControl aerial_manipulator_control(rate, simulation_flag);
    aerial_manipulator_control.LoadParameters(path+parameters_file);

    ros::Subscriber clock_ros_sub = n.subscribe("/clock", 1, &AerialManipulatorControl::clockCb, &aerial_manipulator_control);
    ros::Subscriber uav_pose_meas_odometry_sub = n.subscribe("aerial_manipulator_control/uav_odometry_meas_input", 1, &AerialManipulatorControl::uavPoseMeasOdomCb, &aerial_manipulator_control);
    ros::Subscriber arm_pose_meas_sub = n.subscribe("aerial_manipulator_control/arm_pose_meas_input", 1, &AerialManipulatorControl::armPoseMeasOdomCb, &aerial_manipulator_control);
    
    ros::Publisher transformation_pub_ = n.advertise<std_msgs::Float64MultiArray>("aerial_manipulator_control/transformation/world_end_effector", 1);

    while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start");
    }
    ROS_INFO("Received first clock message");

    while (!aerial_manipulator_control.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Waiting aerial manipulator control to start...");

    time_old = aerial_manipulator_control.getTime().toSec();

    while (ros::ok()) {
        ros::spinOnce();

        time = aerial_manipulator_control.getTime().toSec();

        dt = time - time_old;
        time_old = time;

        if (dt > 0.0) {
            aerial_manipulator_control.getEndEffectorTransformationMatrix(Tworld_end_effector); 

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    transformation_msg.data[j + i*4] = Tworld_end_effector(i, j);
                }
            }
        }

        transformation_pub_.publish(transformation_msg);

        loop_rate.sleep();
    }

    return 0;
}