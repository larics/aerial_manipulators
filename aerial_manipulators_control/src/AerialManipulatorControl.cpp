#include "ros/ros.h"
#include <ros/package.h>

#include <impedance_control/impedance_control.h>
#include <impedance_control/aic.h>

#include <aerial_manipulators_control/ManipulatorControl.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

class AerialManipulatorControl {
    private:
        void initializeImpedanceFilterTransferFunction() {
            impedance_control_x_.initializeImpedanceFilterTransferFunction();
            impedance_control_y_.initializeImpedanceFilterTransferFunction();
            impedance_control_z_.initializeImpedanceFilterTransferFunction();
        };

        void setImpedanceFilterInitialValue(double *initial_values) {
            impedance_control_x_.setImpedanceFilterInitialValue(initial_values[0]);
            impedance_control_y_.setImpedanceFilterInitialValue(initial_values[1]);
            impedance_control_z_.setImpedanceFilterInitialValue(initial_values[2]);
        };

        void initializeAdaptationLaws()
        {
            aic_control_x_.initializeAdaptationLaws();
            aic_control_y_.initializeAdaptationLaws();
            aic_control_z_.initializeAdaptationLaws();
        };

        void setAdaptiveEnvironmentStiffnessInitialValue()
        {
            aic_control_x_.setAdaptiveParameterInitialValues(kp0_[0]);
            aic_control_y_.setAdaptiveParameterInitialValues(kp0_[1]);
            aic_control_z_.setAdaptiveParameterInitialValues(kp0_[2]);
        };

        void fetchEndEffectorPosition() {
            float position[3], q[4], orientationEuler[3];

            arm_pose_meas_ = wp_control.getEndEffectorPosition();

            position[0] = arm_pose_meas_.pose.position.x;
            position[1] = arm_pose_meas_.pose.position.y;
            position[2] = arm_pose_meas_.pose.position.z;

            q[0] = arm_pose_meas_.pose.orientation.w;
            q[1] = arm_pose_meas_.pose.orientation.x;
            q[2] = arm_pose_meas_.pose.orientation.y;
            q[3] = arm_pose_meas_.pose.orientation.z;

            quaternion2euler(q, orientationEuler);

            getRotationTranslationMatrix(Tarm_end_effector_, orientationEuler, position);
            Tend_effector_arm_ = Tarm_end_effector_.inverse();
        };

        int rate_;
        bool simulation_flag_, uav_pose_meas_received_;
        bool start_flag_, force_msg_received_;
        double *xr_, *xc_, *yr_, *yc_, *zr_, *zc_, *xKp_, *yKp_, *zKp_;
        double qxc_[3], qyc_[3], qzc_[3], qwc_[3];
        double xq_, yq_, zq_;

        geometry_msgs::PoseStamped uav_pose_meas_, arm_pose_meas_;
        geometry_msgs::PoseStamped pose_ref_;
        geometry_msgs::TwistStamped vel_ref_, acc_ref_;
        geometry_msgs::WrenchStamped force_meas_, force_torque_ref_;

        Eigen::Matrix4d Tuav_arm_, Tworld_uav_origin_, Tuav_origin_world_;
        Eigen::Matrix4d Tend_effector_arm_, Tarm_end_effector_;

        std::vector<double> kp1_, kp2_, wp_, wd_, M_, B_, K_, dead_zone_, kp0_;

        rosgraph_msgs::Clock clock_;

        ImpedanceControl impedance_control_z_, impedance_control_y_, impedance_control_x_;
        aic aic_control_z_, aic_control_x_, aic_control_y_;
        ManipulatorControl wp_control;
    public:
        AerialManipulatorControl(int rate, bool simulation):
            rate_(rate),
            simulation_flag_(simulation),
            uav_pose_meas_received_(false),
            force_msg_received_(false),
            start_flag_(false),
            impedance_control_x_(rate),
            impedance_control_y_(rate),
            impedance_control_z_(rate),
            aic_control_x_(rate),
            aic_control_y_(rate),
            aic_control_z_(rate),
            qxc_(),
            qyc_(),
            qzc_(),
            qwc_(),
            xq_(0.0),
            yq_(0.0),
            zq_(0.0) {

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

        void forceTorqueRefCb(const geometry_msgs::WrenchStamped &msg) {
            force_torque_ref_ = msg;
        };

        void forceMeasurementCb(const geometry_msgs::WrenchStamped &msg) {
            force_msg_received_ = true;
            force_meas_ = msg;
        };

        void poseRefCb(const geometry_msgs::PoseStamped &msg) {
            pose_ref_ = msg;
            vel_ref_ = geometry_msgs::TwistStamped();
            acc_ref_ = geometry_msgs::TwistStamped();
        };

        bool startControlCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res) {
            bool service_flag = false;
            double initial_values[3];

            if (req.data && isReady() && !start_flag_)
            {
                //initial_values[0] = pose_meas_.pose.position.x;
                //initial_values[1] = pose_meas_.pose.position.y;
                //initial_values[2] = pose_meas_.pose.position.z;

                pose_ref_.pose.position.x = initial_values[0];
                pose_ref_.pose.position.y = initial_values[1];
                pose_ref_.pose.position.z = initial_values[2];
                pose_ref_.pose.orientation.x = 0;//pose_meas_.pose.orientation.x;
                pose_ref_.pose.orientation.y = 0;//pose_meas_.pose.orientation.y;
                pose_ref_.pose.orientation.z = 0;//pose_meas_.pose.orientation.z;
                pose_ref_.pose.orientation.w = 0;//pose_meas_.pose.orientation.w;

                vel_ref_.twist.linear.x = 0;
                vel_ref_.twist.linear.y = 0;
                vel_ref_.twist.linear.z = 0;
                vel_ref_.twist.angular.x = 0;
                vel_ref_.twist.angular.y = 0;
                vel_ref_.twist.angular.z = 0;

                acc_ref_.twist.linear.x = 0;
                acc_ref_.twist.linear.y = 0;
                acc_ref_.twist.linear.z = 0;
                acc_ref_.twist.angular.x = 0;
                acc_ref_.twist.angular.y = 0;
                acc_ref_.twist.angular.z = 0;

                ROS_INFO("Starting aerial manipulator control.");
                initializeImpedanceFilterTransferFunction();
                setImpedanceFilterInitialValue(initial_values);

                initializeAdaptationLaws();
                setAdaptiveEnvironmentStiffnessInitialValue();

                start_flag_ = true;
                service_flag = true;
            }
            else if (!req.data)
            {
                ROS_INFO("Stoping aerial manipulator control.");
                start_flag_ = false;
                uav_pose_meas_received_ = false;
                force_msg_received_ = false;
                service_flag = true;
            }

            res.success = service_flag;

            return true;
        };

        void LoadParameters(std::string file) {
            YAML::Node config = YAML::LoadFile(file);
            std::vector<double> manipulator_origin;

            manipulator_origin = config["origin"]["manipulator"].as<std::vector<double> >();

            Tuav_arm_ <<  cos(manipulator_origin[5])*cos(manipulator_origin[4]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])-sin(manipulator_origin[5])*cos(manipulator_origin[3]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])+sin(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[0],
                          sin(manipulator_origin[5])*cos(manipulator_origin[4]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])+cos(manipulator_origin[5])*cos(manipulator_origin[3]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])-cos(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[1],
                         -sin(manipulator_origin[4]),                             cos(manipulator_origin[4])*sin(manipulator_origin[3]),                                                                                    cos(manipulator_origin[4])*cos(manipulator_origin[3]),                                                                                  manipulator_origin[2],
                          0,                                                      0,                                                                                                                                        0,                                                                                                                                      1;
        
            M_ = config["IMPEDANCE_FILTER"]["M"].as<std::vector<double> >();
            B_ = config["IMPEDANCE_FILTER"]["B"].as<std::vector<double> >();
            K_ = config["IMPEDANCE_FILTER"]["K"].as<std::vector<double> >();
            dead_zone_ = config["IMPEDANCE_FILTER"]["dead_zone"].as<std::vector<double> >();
            kp1_ = config["AIC"]["INTEGRAL_ADAPTATION_GAINS"]["ke1"].as<std::vector<double> >();
            kp2_ = config["AIC"]["PROPORTIONAL_ADAPTATION_GAINS"]["ke2"].as<std::vector<double> >();
            kp0_ = config["AIC"]["INITIAL_GAINS"]["Ke"].as<std::vector<double> >();
            wp_ = config["AIC"]["WEIGHTING_FACTORS"]["Wp"].as<std::vector<double> >();
            wd_ = config["AIC"]["WEIGHTING_FACTORS"]["Wd"].as<std::vector<double> >();
            
            impedance_control_x_.setImpedanceFilterMass(M_[0]);
            impedance_control_x_.setImpedanceFilterDamping(B_[0]);
            impedance_control_x_.setImpedanceFilterStiffness(K_[0]);
            impedance_control_x_.setDeadZone(dead_zone_[0]);
            aic_control_x_.setImpedanceFilterParameters(M_[0], B_[0], K_[0]);
            aic_control_x_.setAdaptiveParameters(kp1_[0], kp2_[0], wp_[0], wd_[0]);
            aic_control_x_.setDeadZone(dead_zone_[0]);


            impedance_control_y_.setImpedanceFilterMass(M_[1]);
            impedance_control_y_.setImpedanceFilterDamping(B_[1]);
            impedance_control_y_.setImpedanceFilterStiffness(K_[1]);
            impedance_control_y_.setDeadZone(dead_zone_[1]);
            aic_control_y_.setImpedanceFilterParameters(M_[1], B_[1], K_[1]);
            aic_control_y_.setAdaptiveParameters(kp1_[1], kp2_[1], wp_[1], wd_[1]);
            aic_control_y_.setDeadZone(dead_zone_[1]);


            impedance_control_z_.setImpedanceFilterMass(M_[2]);
            impedance_control_z_.setImpedanceFilterDamping(B_[2]);
            impedance_control_z_.setImpedanceFilterStiffness(K_[2]);
            impedance_control_z_.setDeadZone(dead_zone_[2]);
            aic_control_z_.setImpedanceFilterParameters(M_[2], B_[2], K_[2]);
            aic_control_z_.setAdaptiveParameters(kp1_[2], kp2_[2], wp_[2], wd_[2]);
            aic_control_z_.setDeadZone(dead_zone_[2]);

            wp_control.LoadParameters(file);
        };

        bool isStarted(void) {
            return start_flag_;
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
            this->fetchEndEffectorPosition();
            transformationMatrix = Tworld_uav_origin_ * Tuav_arm_ * Tarm_end_effector_;
        };

        bool isReady(void) {
            return uav_pose_meas_received_ && force_msg_received_ && wp_control.isStarted();
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

        void setManipulatorName(std::string robot_model_name, std::string joint_model_group_name) {
            wp_control.setManipulatorName(robot_model_name, joint_model_group_name);
        }

        void manipulatorInit(ros::NodeHandle *n) {
            wp_control.init(n);
        }

        void impedanceFilter() {
            double xd[3], yd[3], zd[3];

            xd[0] = pose_ref_.pose.position.x;
            xd[1] = vel_ref_.twist.linear.x;
            xd[2] = acc_ref_.twist.linear.x;

            yd[0] = pose_ref_.pose.position.y;
            yd[1] = vel_ref_.twist.linear.y;
            yd[2] = acc_ref_.twist.linear.y;

            zd[0] = pose_ref_.pose.position.z;
            zd[1] = vel_ref_.twist.linear.z;
            zd[2] = acc_ref_.twist.linear.z;

            qxc_[0] = pose_ref_.pose.orientation.x;
            qxc_[1] = 0.0;
            qxc_[2] = 0.0;

            qyc_[0] = pose_ref_.pose.orientation.y;
            qyc_[1] = 0.0;
            qyc_[2] = 0.0;

            qzc_[0] = pose_ref_.pose.orientation.z;
            qzc_[1] = 0.0;
            qzc_[2] = 0.0;

            qwc_[0] = pose_ref_.pose.orientation.w;
            qwc_[1] = 0.0;
            qwc_[2] = 0.0;


            xr_ = aic_control_x_.compute(force_meas_.wrench.force.x, force_torque_ref_.wrench.force.x, xd);
            xKp_ = aic_control_x_.getAdaptiveEnvironmentStiffnessGainKp();
            xq_ = aic_control_x_.getQ();
            xc_ = impedance_control_x_.impedanceFilter(force_meas_.wrench.force.x, force_torque_ref_.wrench.force.x, xr_);

            yr_ = aic_control_y_.compute(force_meas_.wrench.force.y, force_torque_ref_.wrench.force.y, yd);
            yKp_ = aic_control_y_.getAdaptiveEnvironmentStiffnessGainKp();
            yq_ = aic_control_y_.getQ();
            yc_ = impedance_control_y_.impedanceFilter(force_meas_.wrench.force.y, force_torque_ref_.wrench.force.y, yr_);

            zr_ = aic_control_z_.compute(force_meas_.wrench.force.z, force_torque_ref_.wrench.force.z, zd);
            zKp_ = aic_control_z_.getAdaptiveEnvironmentStiffnessGainKp();
            zq_ = aic_control_z_.getQ();
            zc_ = impedance_control_z_.impedanceFilter(force_meas_.wrench.force.z, force_torque_ref_.wrench.force.z, zr_);
        };

        double *getXc() {
            return xc_;
        };

        double *getYc() {
            return yc_;
        };

        double *getZc() {
            return zc_;
        };

        double *getQXc() {
            return qxc_;
        };

        double *getQYc() {
            return qyc_;
        };

        double *getQZc() {
            return qzc_;
        };

        double *getQWc() {
            return qwc_;
        };

        double *getXr() {
            return xr_;
        };

        double *getYr() {
            return yr_;
        };

        double *getZr() {
            return zr_;
        };

        double *getXKp() {
            return xKp_;
        };

        double *getYKp() {
            return yKp_;
        };

        double *getZKp() {
            return zKp_;
        };

        double getXq() {
            return xq_;
        };

        double getYq() {
            return yq_;
        };

        double getZq() {
            return zq_;
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
    std::string robot_model_name, joint_group_name;
    std::string parameters_file;

    ros::init(argc, argv, "aerial_manipulator_control");
    ros::NodeHandle n, private_node_handle_("~");

    private_node_handle_.param("rate", rate, int(100));
    private_node_handle_.param("simulation", simulation_flag, bool(true));
    private_node_handle_.param("robot_name", robot_model_name, std::string("wp_manipulator"));
    private_node_handle_.param("joint_group_name", joint_group_name, std::string("wp_manipulator_arm"));
    private_node_handle_.param("parameters_file", parameters_file, std::string("/config/aerial_manipulator_parameters.yaml"));

    ros::Rate loop_rate(rate);

    AerialManipulatorControl aerial_manipulator_control(rate, simulation_flag);
    aerial_manipulator_control.setManipulatorName(robot_model_name, joint_group_name);
    aerial_manipulator_control.LoadParameters(path+parameters_file);
    aerial_manipulator_control.manipulatorInit(&n);

    ros::Subscriber clock_ros_sub = n.subscribe("/clock", 1, &AerialManipulatorControl::clockCb, &aerial_manipulator_control);
    ros::Subscriber uav_pose_meas_odometry_sub = n.subscribe("aerial_manipulator_control/uav_odometry_meas_input", 1, &AerialManipulatorControl::uavPoseMeasOdomCb, &aerial_manipulator_control);
    ros::Subscriber pose_ref_sub = n.subscribe("aerial_manipulator_control/pose_stamped_ref_input", 1, &AerialManipulatorControl::poseRefCb, &aerial_manipulator_control);  
    ros::Subscriber force_torque_ref_ros_sub = n.subscribe("aerial_manipulator_control/force_torque_ref_input", 1, &AerialManipulatorControl::forceTorqueRefCb, &aerial_manipulator_control);
    ros::Subscriber force_ros_sub = n.subscribe("aerial_manipulator_control/force_torque_meas_input", 1, &AerialManipulatorControl::forceMeasurementCb, &aerial_manipulator_control);
    
    ros::Publisher transformation_pub_ = n.advertise<std_msgs::Float64MultiArray>("aerial_manipulator_control/transformation/world_end_effector", 1);

    ros::ServiceServer start_control_ros_srv = n.advertiseService("aerial_manipulator_control/start", &AerialManipulatorControl::startControlCb, &aerial_manipulator_control);

    while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start 3");
        ros::Duration(0.5).sleep();
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

            if (aerial_manipulator_control.isStarted()) {

                aerial_manipulator_control.impedanceFilter();
            }
        }

        transformation_pub_.publish(transformation_msg);

        loop_rate.sleep();
    }

    return 0;
}