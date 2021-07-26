#include "ros/ros.h"
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

#include <impedance_control/impedance_control.h>
#include <impedance_control/aic.h>

#include <aerial_manipulators_control/ManipulatorControl.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

#include <aerial_manipulators_control/ManipulatorControl.h>
#include <aerial_manipulators_control/ImpedanceControlConfig.h>

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

        void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, float *angles) {
          double r11, r21, r31, r32, r33;
          double roll, pitch, yaw;

          r11 = rotationTranslationMatrix(0,0);
          r21 = rotationTranslationMatrix(1,0);
          r31 = rotationTranslationMatrix(2,0);
          r32 = rotationTranslationMatrix(2,1);
          r33 = rotationTranslationMatrix(2,2);

          roll = atan2(r32, r33);
          pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
          yaw = atan2(r21, r11);

          angles[0] = roll;
          angles[1] = pitch;
          angles[2] = yaw;
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

        double norm(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
            double n, dx, dy, dz;

            dx =  p1.position.x - p2.position.x;
            dy =  p1.position.y - p2.position.y;
            dz =  p1.position.z - p2.position.z;

            n = sqrt(dx*dx + dy*dy + dz*dz);

            return n;
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

        void euler2quaternion(float *euler, float *quaternion) {
            float cy = cos(euler[2] * 0.5);
            float sy = sin(euler[2] * 0.5);
            float cr = cos(euler[0] * 0.5);
            float sr = sin(euler[0] * 0.5);
            float cp = cos(euler[1] * 0.5);
            float sp = sin(euler[1] * 0.5);

            quaternion[0] = cy * cr * cp + sy * sr * sp; //w
            quaternion[1] = cy * sr * cp - sy * cr * sp; //x
            quaternion[2] = cy * cr * sp + sy * sr * cp; //y
            quaternion[3] = sy * cr * cp - cy * sr * sp; //z
        };

        void fetchLocalEndEffectorPosition() {
            float position[3], q[4], orientationEuler[3];

            manipulator_pose_meas_ = manipulator_control_.getEndEffectorPosition();

            position[0] = manipulator_pose_meas_.pose.position.x;
            position[1] = manipulator_pose_meas_.pose.position.y;
            position[2] = manipulator_pose_meas_.pose.position.z;

            q[0] = manipulator_pose_meas_.pose.orientation.w;
            q[1] = manipulator_pose_meas_.pose.orientation.x;
            q[2] = manipulator_pose_meas_.pose.orientation.y;
            q[3] = manipulator_pose_meas_.pose.orientation.z;

            quaternion2euler(q, orientationEuler);

            getRotationTranslationMatrix(Tarm_end_effector_, orientationEuler, position);
            Tend_effector_arm_ = Tarm_end_effector_.inverse();
        };

        void aerial_manipulator_inverse_kinematics() {
            double alpha = 0.0;
            double q_norm = 0.0;
            bool ik_found = false;
            geometry_msgs::Pose manipulator_command_pose, uav_command_pose;
            geometry_msgs::PoseStamped manipulator_position_ref;
            Eigen::Vector4d dPuav, dPmanipulator, dPmanipulator_local;
            Eigen::VectorXd dP(4);
            Eigen::MatrixXd J_manipulator, J_uav(6, 3);
            Eigen::Matrix4d Tarm_end_effector_ref, Tend_effector_arm_ref;
            Eigen::Matrix4d Tworld_end_effector_ref, Tend_effector_world_ref;
            Eigen::Matrix4d Tworld_uav_origin_ref;
            std::vector<double> q_manipulator_setpoint;
            float manipulator_position[3], manipulator_orientation_euler[3], manipulator_orientation_q[4];
            float aerial_manipulator_position[3], aerial_manipulator_orientation_q[4], aerial_manipulator_orientation_euler[3];
            float orientationEuler[3], orientationQuaternion[4];

            dP(0) = aerial_manipulator_command_pose_[0].pose.position.x - aerial_manipulator_command_pose_[1].pose.position.x;
            dP(1) = aerial_manipulator_command_pose_[0].pose.position.y - aerial_manipulator_command_pose_[1].pose.position.y;
            dP(2) = aerial_manipulator_command_pose_[0].pose.position.z - aerial_manipulator_command_pose_[1].pose.position.z;
            dP(3) = 0.0;

            //pozicija iz impedancije u transformacijsku matricu
            aerial_manipulator_position[0] = aerial_manipulator_command_pose_[0].pose.position.x;
            aerial_manipulator_position[1] = aerial_manipulator_command_pose_[0].pose.position.y;
            aerial_manipulator_position[2] = aerial_manipulator_command_pose_[0].pose.position.z;
            aerial_manipulator_orientation_q[0] = aerial_manipulator_command_pose_[0].pose.orientation.w;
            aerial_manipulator_orientation_q[1] = aerial_manipulator_command_pose_[0].pose.orientation.x;
            aerial_manipulator_orientation_q[2] = aerial_manipulator_command_pose_[0].pose.orientation.y;
            aerial_manipulator_orientation_q[3] = aerial_manipulator_command_pose_[0].pose.orientation.z;
            quaternion2euler(aerial_manipulator_orientation_q, aerial_manipulator_orientation_euler);
            getRotationTranslationMatrix(Tworld_end_effector_ref, aerial_manipulator_orientation_euler, aerial_manipulator_position);
            Tend_effector_world_ref = Tworld_end_effector_ref.inverse();

            //direktna za manipulator
            manipulator_position_ref = manipulator_control_.getEndEffectorPositionFromQ(q_manipulator_ref_);
            manipulator_position[0] = manipulator_position_ref.pose.position.x;
            manipulator_position[1] = manipulator_position_ref.pose.position.y;
            manipulator_position[2] = manipulator_position_ref.pose.position.z;
            manipulator_orientation_q[0] = manipulator_position_ref.pose.orientation.w;
            manipulator_orientation_q[1] = manipulator_position_ref.pose.orientation.x;
            manipulator_orientation_q[2] = manipulator_position_ref.pose.orientation.y;
            manipulator_orientation_q[3] = manipulator_position_ref.pose.orientation.z;
            quaternion2euler(manipulator_orientation_q, manipulator_orientation_euler);
            getRotationTranslationMatrix(Tarm_end_effector_ref, manipulator_orientation_euler, manipulator_position);
            Tend_effector_arm_ref = Tarm_end_effector_ref.inverse();


            Tworld_uav_origin_ref = Tworld_end_effector_ref * Tend_effector_arm_ref * Tarm_uav_;
            getAnglesFromRotationTranslationMatrix(Tworld_uav_origin_ref, orientationEuler);
            euler2quaternion(orientationEuler, orientationQuaternion);

            //calculate norm distance 
            //q_manipulator_meas = manipulator_control_.getJointMeasurements();

            //dPmanipulator_local = Tarm_uav_ * Tuav_origin_world_ * dP;
            //dPmanipulator_local(1) = 0.0;
            //dPmanipulator_local(2) = 0.0;
            //dPmanipulator_local(3) = 0.0;

            //manipulator_command_pose.position.x = manipulator_command_pose_.pose.position.x + dPmanipulator_local(0);
            //manipulator_command_pose.position.y = manipulator_command_pose_.pose.position.y + dPmanipulator_local(1);
            //manipulator_command_pose.position.z = manipulator_command_pose_.pose.position.z + dPmanipulator_local(2);
            //manipulator_command_pose.orientation.x = manipulator_command_pose_.pose.orientation.x;
            //manipulator_command_pose.orientation.y = manipulator_command_pose_.pose.orientation.y;
            //manipulator_command_pose.orientation.z = manipulator_command_pose_.pose.orientation.z;
            //manipulator_command_pose.orientation.w = manipulator_command_pose_.pose.orientation.w;

            //q_manipulator_setpoint_ = manipulator_q_home_;//manipulator_control_.calculateJointSetpoints(manipulator_command_pose, ik_found, 10, 0.01);

            /*for (int i = 0; i < manipulator_q_home_.size(); i++) {
                q_norm += pow(q_manipulator_setpoint[i] - manipulator_q_home_[i], 2);
            }
            q_norm = sqrt(q_norm);

            if (ik_found && q_norm < 0.4) {
                q_manipulator_setpoint_ = q_manipulator_setpoint;
                alpha = 0.0;

                dPmanipulator = Tworld_uav_origin_ * Tuav_arm_ * dPmanipulator_local;
                dPuav = dP - dPmanipulator;
            }
            else {
                alpha = 1.0;
                dPuav = alpha * dP;
                dPmanipulator = (1.0 - alpha) * dP;
                dPmanipulator_local = Tarm_uav_ * Tuav_origin_world_ * dPmanipulator;
            }*/
            q_manipulator_setpoint_ = q_manipulator_ref_;

            uav_command_pose_.header.stamp = this->getTime();
            uav_command_pose_.header.frame_id = "uav";
            uav_command_pose_.pose.position.x = Tworld_uav_origin_ref(0,3);
            uav_command_pose_.pose.position.y = Tworld_uav_origin_ref(1,3);
            uav_command_pose_.pose.position.z = Tworld_uav_origin_ref(2,3);
            uav_command_pose_.pose.orientation.x = orientationQuaternion[1];//aerial_manipulator_command_pose_[0].pose.orientation.x;
            uav_command_pose_.pose.orientation.y = orientationQuaternion[2];//aerial_manipulator_command_pose_[0].pose.orientation.y;
            uav_command_pose_.pose.orientation.z = orientationQuaternion[3];//aerial_manipulator_command_pose_[0].pose.orientation.z;
            uav_command_pose_.pose.orientation.w = orientationQuaternion[0];//aerial_manipulator_command_pose_[0].pose.orientation.w;

            manipulator_command_pose_.header.stamp = this->getTime();
            manipulator_command_pose_.header.frame_id = "manipulator";
            manipulator_command_pose_.pose.position.x = manipulator_position_ref.pose.position.x;
            manipulator_command_pose_.pose.position.y = manipulator_position_ref.pose.position.y;
            manipulator_command_pose_.pose.position.z = manipulator_position_ref.pose.position.z;
            manipulator_command_pose_.pose.orientation.x = manipulator_position_ref.pose.orientation.x;
            manipulator_command_pose_.pose.orientation.y = manipulator_position_ref.pose.orientation.y;
            manipulator_command_pose_.pose.orientation.z = manipulator_position_ref.pose.orientation.z;
            manipulator_command_pose_.pose.orientation.w = manipulator_position_ref.pose.orientation.w;



        }

        void LoadParameters(std::string file) {
            YAML::Node config = YAML::LoadFile(file);
            std::vector<double> manipulator_origin;
            std::vector<int> manipulator_q_directions;

            manipulator_origin = config["manipulator"]["origin"].as<std::vector<double> >();
            manipulator_q_directions = config["manipulator"]["directions"].as<std::vector<int> >();
            manipulator_q_home_ = config["manipulator"]["home"].as<std::vector<double> >();
            manipulator_end_effector_offset_ = config["manipulator"]["end_effector_offset"].as<std::vector<double> >();

            Tuav_arm_ <<  cos(manipulator_origin[5])*cos(manipulator_origin[4]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])-sin(manipulator_origin[5])*cos(manipulator_origin[3]),  cos(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])+sin(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[0],
                    sin(manipulator_origin[5])*cos(manipulator_origin[4]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*sin(manipulator_origin[3])+cos(manipulator_origin[5])*cos(manipulator_origin[3]),  sin(manipulator_origin[5])*sin(manipulator_origin[4])*cos(manipulator_origin[3])-cos(manipulator_origin[5])*sin(manipulator_origin[3]), manipulator_origin[1],
                    -sin(manipulator_origin[4]),                             cos(manipulator_origin[4])*sin(manipulator_origin[3]),                                                                                    cos(manipulator_origin[4])*cos(manipulator_origin[3]),                                                                                  manipulator_origin[2],
                    0,                                                      0,                                                                                                                                        0,                                                                                                                                      1;

            
            Tarm_uav_ = Tuav_arm_.inverse();
            manipulator_control_.set_q_directions(manipulator_q_directions);
            uav_degrees_of_freedom_ = config["UAV"]["degrees_of_freedom"].as<int>();

            M_ = config["IMPEDANCE_FILTER"]["M"].as<std::vector<double> >();
            B_ = config["IMPEDANCE_FILTER"]["B"].as<std::vector<double> >();
            K_ = config["IMPEDANCE_FILTER"]["K"].as<std::vector<double> >();
            dead_zone_ = config["IMPEDANCE_FILTER"]["dead_zone"].as<std::vector<double> >();
            kp1_ = config["AIC"]["INTEGRAL_ADAPTATION_GAINS"]["ke1"].as<std::vector<double> >();
            kp2_ = config["AIC"]["PROPORTIONAL_ADAPTATION_GAINS"]["ke2"].as<std::vector<double> >();
            kp0_ = config["AIC"]["INITIAL_GAINS"]["Ke"].as<std::vector<double> >();
            wp_ = config["AIC"]["WEIGHTING_FACTORS"]["Wp"].as<std::vector<double> >();
            wd_ = config["AIC"]["WEIGHTING_FACTORS"]["Wd"].as<std::vector<double> >();

            lambda_uav_ = config["UAV"]["lambda"].as<double>();
            lambda_manipulator_ = config["manipulator"]["lambda"].as<double>();
            
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
        };

        int rate_, number_of_joints_, uav_degrees_of_freedom_;
        bool simulation_flag_, uav_pose_meas_received_;
        bool start_flag_, force_msg_received_, reconfigure_start_;
        double *xr_, *xc_, *yr_, *yc_, *zr_, *zc_, *xKp_, *yKp_, *zKp_;
        double qxc_[3], qyc_[3], qzc_[3], qwc_[3];
        double xq_, yq_, zq_, lambda_manipulator_, lambda_uav_;
        double beta_;

        geometry_msgs::PoseStamped uav_pose_meas_, manipulator_pose_meas_, uav_command_pose_, manipulator_command_pose_;
        geometry_msgs::PoseStamped uav_pose_ref_, pose_ref_, pose_meas_, aerial_manipulator_command_pose_[2];
        geometry_msgs::TwistStamped vel_ref_, acc_ref_, uav_vel_ref_, uav_acc_ref_;
        geometry_msgs::WrenchStamped force_meas_, force_torque_ref_;

        Eigen::Matrix4d Tuav_arm_, Tworld_uav_origin_, Tuav_origin_world_, Tarm_uav_;
        Eigen::Matrix4d Tend_effector_arm_, Tarm_end_effector_;
        Eigen::Matrix4d Tworld_end_effector_, Tend_effector_world_;

        std::vector<double> kp1_, kp2_, wp_, wd_, M_, B_, K_, dead_zone_, kp0_;
        std::vector<double> q_manipulator_setpoint_, q_manipulator_ref_;
        std::vector<double> manipulator_q_home_, manipulator_end_effector_offset_;

        rosgraph_msgs::Clock clock_;

        ManipulatorControl manipulator_control_;
        ImpedanceControl impedance_control_z_, impedance_control_y_, impedance_control_x_;
        aic aic_control_z_, aic_control_x_, aic_control_y_;

    public:
        AerialManipulatorControl(int rate, bool simulation):
            rate_(rate),
            simulation_flag_(simulation),
            uav_pose_meas_received_(false),
            number_of_joints_(0),
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
            zq_(0.0),
            reconfigure_start_(false),
            beta_(1.0) {

            Tuav_arm_ <<  1,  0,  0,  0,
                          0,  1,  0,  0,
                          0,  0,  1,  0,
                          0,  0,  0,  1;

            Tarm_uav_ = Tuav_arm_.inverse();

            Tworld_uav_origin_ << 1, 0, 0, 0,
                                  0, 1, 0, 0, 
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;

            Tuav_origin_world_ << 1, 0, 0, 0,
                                  0, 1, 0, 0, 
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;
        };

        void initManipulatorControl(ros::NodeHandle *n, std::string robot_model_name, std::string joint_model_group_name, std::string param_file) {
            LoadParameters(param_file);

            manipulator_control_.setManipulatorName(robot_model_name, joint_model_group_name);
            manipulator_control_.init(n);

            number_of_joints_ = manipulator_control_.getNumberOfJoints();

            q_manipulator_setpoint_ = std::vector<double>(number_of_joints_, 0);
            q_manipulator_ref_ = std::vector<double>(number_of_joints_, 0);
        };

        void reconfigureCb(aerial_manipulators_control::ImpedanceControlConfig &config, uint32_t level) {

            if (!reconfigure_start_) {
                config.kp1_x = kp1_[0];
                config.kp2_x = kp2_[0];
                config.wp_x = wp_[0];
                config.wd_x = wd_[0];

                config.kp1_y = kp1_[1];
                config.kp2_y = kp2_[1];
                config.wp_y = wp_[1];
                config.wd_y = wd_[1];

                config.kp1_z = kp1_[2];
                config.kp2_z = kp2_[2];
                config.wp_z = wp_[2];
                config.wd_z = wd_[2];
                reconfigure_start_ = true;
            }
            else {
                kp1_[0] = config.kp1_x;
                kp2_[0] = config.kp2_x;
                wp_[0] = config.wp_x;
                wd_[0] = config.wd_x;

                kp1_[1] = config.kp1_y;
                kp2_[1] = config.kp2_y;
                wp_[1] = config.wp_y;
                wd_[1] = config.wd_y;

                kp1_[2] = config.kp1_z;
                kp2_[2] = config.kp2_z;
                wp_[2] = config.wp_z;
                wd_[2] = config.wd_z;

                aic_control_x_.setAdaptiveParameters(kp1_[0], kp2_[0], wp_[0], wd_[0]);
                aic_control_y_.setAdaptiveParameters(kp1_[1], kp2_[1], wp_[1], wd_[1]);
                aic_control_z_.setAdaptiveParameters(kp1_[2], kp2_[2], wp_[2], wd_[2]);

                initializeAdaptationLaws();
            }   
        };

        void trajectoryRefCb(const trajectory_msgs::JointTrajectoryPoint &msg) {
            float orientationQuaternion[4], orientationEuler[3];
            float manipulator_position[3], manipulator_orientation_euler[3], manipulator_orientation_q[4];
            float uav_position[3], uav_orientation_euler[3], uav_orientation_q[4];
            geometry_msgs::PoseStamped manipulator_position_ref;
            Eigen::Matrix4d Tarm_end_effector_ref, Tend_effector_arm_ref;
            Eigen::Matrix4d Tuav_origin_world_ref, Tworld_uav_origin_ref;
            Eigen::Matrix4d Tend_effector_world_ref, Tworld_end_effector_ref;
            if (msg.positions.size() >= (number_of_joints_+ 13))
            {
                uav_pose_ref_.header.stamp = this->getTime();
                uav_pose_ref_.header.frame_id = "uav";
                uav_pose_ref_.pose.position.x = msg.positions[0];
                uav_pose_ref_.pose.position.y = msg.positions[1];
                uav_pose_ref_.pose.position.z = msg.positions[2];
                orientationEuler[0] = 0.0;//msg.positions[3];
                orientationEuler[1] = 0.0;//msg.positions[4];
                orientationEuler[2] = msg.positions[5];
                //std::cout<<uav_pose_ref_<<std::endl;

                euler2quaternion(orientationEuler, orientationQuaternion);

                uav_pose_ref_.pose.orientation.x = orientationQuaternion[1];
                uav_pose_ref_.pose.orientation.y = orientationQuaternion[2];
                uav_pose_ref_.pose.orientation.z = orientationQuaternion[3];
                uav_pose_ref_.pose.orientation.w = orientationQuaternion[0];

                uav_vel_ref_.header.stamp = this->getTime();
                uav_vel_ref_.header.frame_id = "uav";
                uav_vel_ref_.twist.linear.x = msg.velocities[0];
                uav_vel_ref_.twist.linear.y = msg.velocities[1];
                uav_vel_ref_.twist.linear.z = msg.velocities[2];
                uav_vel_ref_.twist.angular.x = 0.0;//msg.velocities[3];
                uav_vel_ref_.twist.angular.y = 0.0;//msg.velocities[4];
                uav_vel_ref_.twist.angular.z = msg.velocities[5];

                uav_acc_ref_.header.stamp = this->getTime();
                uav_acc_ref_.header.frame_id = "uav";
                uav_acc_ref_.twist.linear.x = msg.accelerations[0];
                uav_acc_ref_.twist.linear.y = msg.accelerations[1];
                uav_acc_ref_.twist.linear.z = msg.accelerations[2];
                uav_acc_ref_.twist.angular.x = 0.0;//msg.accelerations[3];
                uav_acc_ref_.twist.angular.y = 0.0;//msg.accelerations[4];
                uav_acc_ref_.twist.angular.z = msg.accelerations[5];

                for (int i = 0; i < number_of_joints_; i++)
                {
                    q_manipulator_ref_[i] = msg.positions[i+6];
                }

                force_torque_ref_.header.stamp = this->getTime();
                force_torque_ref_.header.frame_id = "aerial_manipulator";
                force_torque_ref_.wrench.force.x = msg.positions[number_of_joints_+ 6];
                force_torque_ref_.wrench.force.y = msg.positions[number_of_joints_+ 7];
                force_torque_ref_.wrench.force.z = msg.positions[number_of_joints_+ 8];
                force_torque_ref_.wrench.torque.x = msg.positions[number_of_joints_+ 9];
                force_torque_ref_.wrench.torque.y = msg.positions[number_of_joints_+ 10];
                force_torque_ref_.wrench.torque.z = msg.positions[number_of_joints_+ 11];

                beta_ = msg.positions[number_of_joints_+ 12];


                //TODO pretvoriti to u vrh alata
                manipulator_position_ref = manipulator_control_.getEndEffectorPositionFromQ(q_manipulator_ref_);
                manipulator_position[0] = manipulator_position_ref.pose.position.x;
                manipulator_position[1] = manipulator_position_ref.pose.position.y;
                manipulator_position[2] = manipulator_position_ref.pose.position.z;
                manipulator_orientation_q[0] = manipulator_position_ref.pose.orientation.w;
                manipulator_orientation_q[1] = manipulator_position_ref.pose.orientation.x;
                manipulator_orientation_q[2] = manipulator_position_ref.pose.orientation.y;
                manipulator_orientation_q[3] = manipulator_position_ref.pose.orientation.z;
                quaternion2euler(manipulator_orientation_q, manipulator_orientation_euler);
                getRotationTranslationMatrix(Tarm_end_effector_ref, manipulator_orientation_euler, manipulator_position);
                Tend_effector_arm_ref = Tarm_end_effector_ref.inverse();


                uav_position[0] = uav_pose_ref_.pose.position.x;
                uav_position[1] = uav_pose_ref_.pose.position.y;
                uav_position[2] = uav_pose_ref_.pose.position.z;
                uav_orientation_q[0] = uav_pose_ref_.pose.orientation.w;
                uav_orientation_q[1] = uav_pose_ref_.pose.orientation.x;
                uav_orientation_q[2] = uav_pose_ref_.pose.orientation.y;
                uav_orientation_q[3] = uav_pose_ref_.pose.orientation.z;
                quaternion2euler(uav_orientation_q, uav_orientation_euler);
                getRotationTranslationMatrix(Tworld_uav_origin_ref, uav_orientation_euler, uav_position);
                Tuav_origin_world_ref = Tworld_uav_origin_ref.inverse();


                Tworld_end_effector_ref = Tworld_uav_origin_ref * Tuav_arm_ * Tarm_end_effector_ref;
                Tend_effector_world_ref = Tworld_end_effector_ref.inverse();
                getAnglesFromRotationTranslationMatrix(Tworld_end_effector_ref, orientationEuler);
                euler2quaternion(orientationEuler, orientationQuaternion);
                pose_ref_.header.stamp = this->getTime();
                pose_ref_.header.frame_id = "aerial_manipulator";
                pose_ref_.pose.position.x = Tworld_end_effector_ref(0,3);
                pose_ref_.pose.position.y = Tworld_end_effector_ref(1,3);
                pose_ref_.pose.position.z = Tworld_end_effector_ref(2,3);
                pose_ref_.pose.orientation.x = orientationQuaternion[1];
                pose_ref_.pose.orientation.y = orientationQuaternion[2];
                pose_ref_.pose.orientation.z = orientationQuaternion[3];
                pose_ref_.pose.orientation.w = orientationQuaternion[0];
            }
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

        void end_effector_cb(const geometry_msgs::PoseStamped &msg) {
            manipulator_command_pose_ = msg;
        }

        void forceMeasurementCb(const geometry_msgs::WrenchStamped &msg) {
            force_msg_received_ = true;
            force_meas_ = msg;
        };

        void poseRefCb(const geometry_msgs::PoseStamped &msg) {
            pose_ref_ = msg;
            vel_ref_ = geometry_msgs::TwistStamped();
            acc_ref_ = geometry_msgs::TwistStamped();
        };

        void uavPoseRefCb(const geometry_msgs::PoseStamped &msg) {

        };

        bool setHomePositionManipulatorCb(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
            q_manipulator_ref_ = manipulator_q_home_;
            q_manipulator_setpoint_ = manipulator_q_home_;
            this->publishManipulatorSetpoints();
            return true;
        };

        bool startControlCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res) {
            bool service_flag = false;
            double initial_values[3];

            if (req.data && isReady() && !start_flag_)
            {
                initial_values[0] = pose_meas_.pose.position.x;
                initial_values[1] = pose_meas_.pose.position.y;
                initial_values[2] = pose_meas_.pose.position.z;

                pose_ref_.header.stamp = this->getTime();
                pose_ref_.header.frame_id = "aerial_manipulator";
                pose_ref_.pose.position.x = initial_values[0];
                pose_ref_.pose.position.y = initial_values[1];
                pose_ref_.pose.position.z = initial_values[2];
                pose_ref_.pose.orientation.x = pose_meas_.pose.orientation.x;
                pose_ref_.pose.orientation.y = pose_meas_.pose.orientation.y;
                pose_ref_.pose.orientation.z = pose_meas_.pose.orientation.z;
                pose_ref_.pose.orientation.w = pose_meas_.pose.orientation.w;

                uav_pose_ref_ = uav_pose_meas_;

                aerial_manipulator_command_pose_[0] = pose_ref_;
                aerial_manipulator_command_pose_[1] = pose_ref_;
                uav_command_pose_ = uav_pose_meas_;
                manipulator_command_pose_ = manipulator_pose_meas_;
                q_manipulator_setpoint_ = manipulator_control_.getJointMeasurements();
                q_manipulator_ref_ = manipulator_control_.getJointMeasurements();

                vel_ref_.header.stamp = this->getTime();
                vel_ref_.header.frame_id = "aerial_manipulator";
                vel_ref_.twist.linear.x = 0;
                vel_ref_.twist.linear.y = 0;
                vel_ref_.twist.linear.z = 0;
                vel_ref_.twist.angular.x = 0;
                vel_ref_.twist.angular.y = 0;
                vel_ref_.twist.angular.z = 0;

                acc_ref_.header.stamp = this->getTime();
                acc_ref_.header.frame_id = "aerial_manipulator";
                acc_ref_.twist.linear.x = 0;
                acc_ref_.twist.linear.y = 0;
                acc_ref_.twist.linear.z = 0;
                acc_ref_.twist.angular.x = 0;
                acc_ref_.twist.angular.y = 0;
                acc_ref_.twist.angular.z = 0;

                uav_vel_ref_.header.stamp = this->getTime();
                uav_vel_ref_.header.frame_id = "uav";
                uav_vel_ref_.twist.linear.x = 0;
                uav_vel_ref_.twist.linear.y = 0;
                uav_vel_ref_.twist.linear.z = 0;
                uav_vel_ref_.twist.angular.x = 0;
                uav_vel_ref_.twist.angular.y = 0;
                uav_vel_ref_.twist.angular.z = 0;

                uav_acc_ref_.header.stamp = this->getTime();
                uav_acc_ref_.header.frame_id = "uav";
                uav_acc_ref_.twist.linear.x = 0;
                uav_acc_ref_.twist.linear.y = 0;
                uav_acc_ref_.twist.linear.z = 0;
                uav_acc_ref_.twist.angular.x = 0;
                uav_acc_ref_.twist.angular.y = 0;
                uav_acc_ref_.twist.angular.z = 0;

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


        bool isStarted(void) {
            return start_flag_;
        };

        void calculateEndEffectorPosition() {
            float orientationEuler[3], orientationQuaternion[4];

            this->fetchLocalEndEffectorPosition();
            Tworld_end_effector_ = Tworld_uav_origin_ * Tuav_arm_ * Tarm_end_effector_;
            Tend_effector_world_ = Tworld_end_effector_.inverse();

            getAnglesFromRotationTranslationMatrix(Tworld_end_effector_, orientationEuler);
            euler2quaternion(orientationEuler, orientationQuaternion);

            pose_meas_.header.stamp = this->getTime();
            pose_meas_.header.frame_id = "aerial_manipulator";
            pose_meas_.pose.position.x = Tworld_end_effector_(0,3);
            pose_meas_.pose.position.y = Tworld_end_effector_(1,3);
            pose_meas_.pose.position.z = Tworld_end_effector_(2,3);
            pose_meas_.pose.orientation.x = orientationQuaternion[1];
            pose_meas_.pose.orientation.y = orientationQuaternion[2];
            pose_meas_.pose.orientation.z = orientationQuaternion[3];
            pose_meas_.pose.orientation.w = orientationQuaternion[0];
        };

        void getEndEffectorTransformationMatrix(Eigen::Matrix4d &transformationMatrix) {
            transformationMatrix = Tworld_end_effector_;
        };

        geometry_msgs::PoseStamped getEndEffectorPositionLocal() {
            return manipulator_pose_meas_;
        }

        geometry_msgs::PoseStamped getAerialManipulatorPose() {
            return pose_meas_;
        }

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

        void impedanceFilter() {
            double xd[3], yd[3], zd[3];

            aerial_manipulator_command_pose_[1] = aerial_manipulator_command_pose_[0];

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

            aerial_manipulator_command_pose_[0].header.stamp = this->getTime();
            aerial_manipulator_command_pose_[0].pose.position.x = xc_[0];
            aerial_manipulator_command_pose_[0].pose.position.y = yc_[0];
            aerial_manipulator_command_pose_[0].pose.position.z = zc_[0];
            aerial_manipulator_command_pose_[0].pose.orientation.x = qxc_[0];
            aerial_manipulator_command_pose_[0].pose.orientation.y = qyc_[0];
            aerial_manipulator_command_pose_[0].pose.orientation.z = qzc_[0];
            aerial_manipulator_command_pose_[0].pose.orientation.w = qwc_[0];

            this->aerial_manipulator_inverse_kinematics();
        };



        geometry_msgs::PoseStamped getAerialManipulatorCommandPose() {
            return aerial_manipulator_command_pose_[0];
        };

        geometry_msgs::PoseStamped getUAVCommandPose() {
            return uav_command_pose_;
        }

        geometry_msgs::PoseStamped getManipulatorCommandPose() {
            return manipulator_command_pose_;
        }

        std::vector<double> getManipulatorCommandQ() {
            return q_manipulator_setpoint_;
        }

        void getState(std_msgs::Float64MultiArray &state_msg) {
            double *xc, *yc, *zc, *xr, *yr, *zr, *xKp, *yKp, *zKp;
            double *qxc, *qyc, *qzc, *qwc;
            double xq, yq, zq;

            state_msg.data.resize(31);

            xc = this->getXc();
            yc = this->getYc();
            zc = this->getZc();
            qxc = this->getQXc();
            qyc = this->getQYc();
            qzc = this->getQZc();
            qwc = this->getQWc();
            xr = this->getXr();
            yr = this->getYr();
            zr = this->getZr();
            xKp = this->getXKp();
            yKp = this->getYKp();
            zKp = this->getZKp();
            xq = this->getXq();
            yq = this->getYq();
            zq = this->getZq();

            int j = 0;
            for (int i = 0; i < 3; i++) {
                state_msg.data[j++] = xr[i];
                state_msg.data[j++] = yr[i];
                state_msg.data[j++] = zr[i];
                state_msg.data[j++] = xc[i];
                state_msg.data[j++] = yc[i];
                state_msg.data[j++] = zc[i];
                state_msg.data[j++] = xKp[i];
                state_msg.data[j++] = yKp[i];
                state_msg.data[j++] = zKp[i];
            }
            state_msg.data[j++] = xq;
            state_msg.data[j++] = yq;
            state_msg.data[j++] = zq;
            state_msg.data[j++] = this->getTime().toSec();
        }

        void publishManipulatorSetpoints() {
            manipulator_control_.publishJointSetpoints(q_manipulator_setpoint_);
        }

        bool isReady(void) {
            return force_msg_received_ && uav_pose_meas_received_ && manipulator_control_.isStarted();
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
    std_msgs::Float64MultiArray state_msg;

    geometry_msgs::PoseStamped commanded_uav_position_msg;
    geometry_msgs::PoseStamped commanded_aerial_manipulator_position_msg;
    geometry_msgs::PoseStamped end_effector_pose, commanded_manipulator_position_msg;
    geometry_msgs::PoseStamped aerial_manipulator_pose;

    transformation_msg.data.resize(16);

    std::string path = ros::package::getPath("aerial_manipulators_control");
    std::string robot_model_name, joint_group_name;
    std::string parameters_file;

    ros::init(argc, argv, "aerial_manipulator_control");
    ros::NodeHandle n, private_node_handle_("~");

    private_node_handle_.param("rate", rate, int(100));
    private_node_handle_.param("simulation", simulation_flag, bool(true));
    private_node_handle_.param("joint_group_name", joint_group_name, std::string("wp_manipulator_arm"));
    private_node_handle_.param("parameters_file", parameters_file, std::string("/config/aerial_manipulator_parameters.yaml"));
    private_node_handle_.param("maniupulator_name", robot_model_name, std::string("wp_manipulator"));

    ros::Rate loop_rate(rate);

    AerialManipulatorControl aerial_manipulator_control(rate, simulation_flag);
    aerial_manipulator_control.initManipulatorControl(&n, robot_model_name, joint_group_name, path+parameters_file);

    ros::Subscriber clock_ros_sub = n.subscribe("/clock", 1, &AerialManipulatorControl::clockCb, &aerial_manipulator_control);
    ros::Subscriber uav_pose_meas_odometry_sub = n.subscribe("aerial_manipulator_control/uav/odometry_meas_input", 1, &AerialManipulatorControl::uavPoseMeasOdomCb, &aerial_manipulator_control);
    ros::Subscriber pose_ref_sub = n.subscribe("aerial_manipulator_control/pose_stamped_ref_input", 1, &AerialManipulatorControl::poseRefCb, &aerial_manipulator_control);
    ros::Subscriber uav_pose_ref_sub = n.subscribe("aerial_manipulator_control/uav/pose_stamped_ref_input", 1, &AerialManipulatorControl::uavPoseRefCb, &aerial_manipulator_control);  
    ros::Subscriber force_torque_ref_ros_sub = n.subscribe("aerial_manipulator_control/force_torque_ref_input", 1, &AerialManipulatorControl::forceTorqueRefCb, &aerial_manipulator_control);
    ros::Subscriber force_ros_sub = n.subscribe("aerial_manipulator_control/force_torque_meas_input", 1, &AerialManipulatorControl::forceMeasurementCb, &aerial_manipulator_control);
    ros::Subscriber aerial_manipulator_trajectory_ref_ros_sub = n.subscribe("aerial_manipulator_control/trajectory_ref_input", 1, &AerialManipulatorControl::trajectoryRefCb, &aerial_manipulator_control);
    ros::Subscriber manipulator_q_ref_sub = n.subscribe("aerial_manipulator_control/end_effector/q_ref", 1, &AerialManipulatorControl::end_effector_cb, &aerial_manipulator_control);


    ros::Publisher transformation_pub_ = n.advertise<std_msgs::Float64MultiArray>("aerial_manipulator_control/transformation/world_end_effector", 1);
    ros::Publisher state_pub_ = n.advertise<std_msgs::Float64MultiArray>("aerial_manipulator_control/state", 1);
    ros::Publisher uav_pose_stamped_commanded_pub_ = n.advertise<geometry_msgs::PoseStamped>("aerial_manipulator_control/uav/pose_stamped_ref_output", 1);
    ros::Publisher uav_pose_commanded_pub_ = n.advertise<geometry_msgs::Pose>("aerial_manipulator_control/uav/pose_ref_output", 1);
    ros::Publisher aerial_manipulator_pose_commanded_pub_ = n.advertise<geometry_msgs::PoseStamped>("aerial_manipulator_control/pose_stamped_ref_output", 1);
    ros::Publisher manipulator_pose_stamped_commanded_pub_ = n.advertise<geometry_msgs::PoseStamped>("aerial_manipulator_control/end_effector/pose_stamped_ref_output", 1);
    ros::Publisher manipulator_position_pub_ros_ = n.advertise<geometry_msgs::PoseStamped>("aerial_manipulator_control/end_effector/pose_output", 1);
    ros::Publisher aerial_manipulator_pose_pub_ros_ = n.advertise<geometry_msgs::PoseStamped>("aerial_manipulator_control/pose_output", 1);

    ros::ServiceServer start_control_ros_srv = n.advertiseService("aerial_manipulator_control/start", &AerialManipulatorControl::startControlCb, &aerial_manipulator_control);
    ros::ServiceServer set_manipulator_home_ros_srv = n.advertiseService("aerial_manipulator_control/manipulator/home", &AerialManipulatorControl::setHomePositionManipulatorCb, &aerial_manipulator_control);

    dynamic_reconfigure::Server<aerial_manipulators_control::ImpedanceControlConfig> server;
    dynamic_reconfigure::Server<aerial_manipulators_control::ImpedanceControlConfig>::CallbackType reconfigure;

    reconfigure = boost::bind(&AerialManipulatorControl::reconfigureCb, &aerial_manipulator_control, _1, _2);
    server.setCallback(reconfigure);

    while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("[AerialManipulatorControl] Waiting for clock server to start");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Received first clock message");

    while (!aerial_manipulator_control.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("[AerialManipulatorControl] Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("[AerialManipulatorControl] Waiting aerial manipulator control to start...");

    time_old = aerial_manipulator_control.getTime().toSec();

    while (ros::ok()) {
        ros::spinOnce();

        time = aerial_manipulator_control.getTime().toSec();

        dt = time - time_old;
        time_old = time;

        if (dt > 0.0) {
            aerial_manipulator_control.calculateEndEffectorPosition();
            aerial_manipulator_control.getEndEffectorTransformationMatrix(Tworld_end_effector);
            end_effector_pose = aerial_manipulator_control.getEndEffectorPositionLocal();
            aerial_manipulator_pose = aerial_manipulator_control.getAerialManipulatorPose();

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    transformation_msg.data[j + i*4] = Tworld_end_effector(i, j);
                }
            }

            if (aerial_manipulator_control.isStarted()) {

                aerial_manipulator_control.impedanceFilter();

                commanded_uav_position_msg = aerial_manipulator_control.getUAVCommandPose();
                uav_pose_stamped_commanded_pub_.publish(commanded_uav_position_msg);
                uav_pose_commanded_pub_.publish(commanded_uav_position_msg.pose);

                aerial_manipulator_control.publishManipulatorSetpoints();

                commanded_manipulator_position_msg = aerial_manipulator_control.getManipulatorCommandPose();
                manipulator_pose_stamped_commanded_pub_.publish(commanded_manipulator_position_msg);

                commanded_aerial_manipulator_position_msg = aerial_manipulator_control.getAerialManipulatorCommandPose();
                aerial_manipulator_pose_commanded_pub_.publish(commanded_aerial_manipulator_position_msg);

                aerial_manipulator_control.getState(state_msg);
                state_pub_.publish(state_msg);
            }

            manipulator_position_pub_ros_.publish(end_effector_pose);
            aerial_manipulator_pose_pub_ros_.publish(aerial_manipulator_pose);
            transformation_pub_.publish(transformation_msg);
        }

        loop_rate.sleep();
    }

    return 0;
}