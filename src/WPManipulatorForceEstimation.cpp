#include <aerial_manipulators/WPManipulatorForceEstimation.h>

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

Eigen::Matrix<float, 6, 1> WPManipulatorForceEstimation::forceEstimate(float *q, float *v0, float *w0, float dtime) {

	WPManipulatorInverseDynamics WPMID(massh_, dch_, D_h_);
	Eigen::Matrix<float, 6, 5> jacob;
	Eigen::Matrix<float, 5, 6> jacobt;
	Eigen::Matrix<float, 5, 1> tauki, taumi;

	taumi = WPMID.calculateID(q, v0, w0, dtime);

	for (int i = 0; i < 5; i++) {
		tauki(i, 0) = tauki_[i];
	}
	jacob = jacobian(q);

	Eigen::Matrix<float, 6, 1> force;
	jacobt = jacob.transpose();
	force = pseudoinverse(jacobt) * (tauki - taumi);

	return force;
}


Eigen::Matrix<float, 6, 5> WPManipulatorForceEstimation::jacobian(float *q) {

	Eigen::Matrix<float, 6, 5> jacob;

  jacob(0, 0) = 0.072489*cos(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*cos(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.075511*cos(q[1] + q[2])*cos(q[3]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*cos(q[1] + q[2]);
	jacob(0, 1) = 0.072489*cos(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.04526*cos(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.075511*cos(q[1] + q[2])*cos(q[3]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*cos(q[1] + q[2]);
	jacob(0, 2) = 0.04526*sin(q[3] + q[4] + q[5])*sin(q[1] + q[2]) + 0.075511*sin(q[1] + q[2])*sin(q[3]) + 0.072489*sin(q[1] + q[2])*cos(q[3])*sin(q[4]) + 0.072489*sin(q[1] + q[2])*cos(q[4])*sin(q[3]);
	jacob(0, 3) = 0.04526*sin(q[3] + q[4] + q[5])*sin(q[1] + q[2]) + 0.072489*sin(q[1] + q[2])*cos(q[3])*sin(q[4]) + 0.072489*sin(q[1] + q[2])*cos(q[4])*sin(q[3]);
	jacob(0, 4) = 0.04526*sin(q[3] + q[4] + q[5])*sin(q[1] + q[2]);

	jacob(1, 0) = 0.072489*sin(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*sin(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*sin(q[1] + q[2]) - 0.075511*sin(q[1] + q[2])*cos(q[3]) - 0.072489*sin(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*sin(q[1] + q[2]);
	jacob(1, 1) = 0.072489*sin(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.04526*cos(q[3] + q[4] + q[5])*sin(q[1] + q[2]) - 0.075511*sin(q[1] + q[2])*cos(q[3]) - 0.072489*sin(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*sin(q[1] + q[2]);
	jacob(1, 2) = -0.04526*sin(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.075511*cos(q[1] + q[2])*sin(q[3]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*sin(q[4]) - 0.072489*cos(q[1] + q[2])*cos(q[4])*sin(q[3]);
	jacob(1, 3) = -0.04526*sin(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*sin(q[4]) - 0.072489*cos(q[1] + q[2])*cos(q[4])*sin(q[3]);
	jacob(1, 4) = -0.04526*sin(q[3] + q[4] + q[5])*cos(q[1] + q[2]);

	jacob(2, 0) = 0;
	jacob(2, 1) = 0;
	jacob(2, 2) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]) - 0.075511*cos(q[3]);
	jacob(2, 3) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]);
	jacob(2, 4) = -0.04526*cos(q[3] + q[4] + q[5]);

	jacob(3, 0) = 0;
	jacob(3, 1) = 0;
	jacob(3, 2) = -1.0*cos(q[1] + q[2]);
	jacob(3, 3) = -1.0*cos(q[1] + q[2]);
	jacob(3, 4) = -1.0*cos(q[1] + q[2]);

	jacob(4, 0) = 0;
	jacob(4, 1) = 0;
	jacob(4, 2) = -1.0*sin(q[1] + q[2]);
	jacob(4, 3) = -1.0*sin(q[1] + q[2]);
	jacob(4, 4) = -1.0*sin(q[1] + q[2]);

	jacob(5, 0) = 1;
	jacob(5, 1) = 1;
	jacob(5, 2) = 0;
	jacob(5, 3) = 0;
	jacob(5, 4) = 0;

	return jacob;
}


WPManipulatorForceEstimation::WPManipulatorForceEstimation(void) {
	sub_ = nh_.subscribe("joint_states", 1, &WPManipulatorForceEstimation::joint_states_Callback, this);
	rate_ = 10;
	pub1_ = nh_.advertise<aerial_manipulators::float6>("force", 1);
  pub2_ = nh_.advertise<aerial_manipulators::float5>("dif_torque", 1);
  pub3_ = nh_.advertise<aerial_manipulators::float5>("est_torque", 1);
}

WPManipulatorForceEstimation::~WPManipulatorForceEstimation(void) {}

void WPManipulatorForceEstimation::joint_states_Callback(const sensor_msgs::JointState &msg)
{
	for (int i = 0; i < 5; i++) {
		q_pos_[i+1] = msg.position[i];
		tauki_[i] = msg.effort[i];
	}
}


void WPManipulatorForceEstimation::run(void)
{

	int count = 0;
	ros::Rate loop_rate(rate_);
	while(ros::ok())
	{
		ros::spinOnce();

		float v0[3] = {0, 0, 0}, w0[3] = {0, 0, 0}, dtime;
		dtime = 1/rate_;
		WPManipulatorInverseDynamics WPMIDtor(massh_, dch_, D_h_);
		if (count == 0) {
			dtime = 0;
		}
		Eigen::Matrix<float, 6, 1> force;
		force = forceEstimate(q_pos_, v0, w0, dtime);
		force_.a = force[0];
		force_.b = force[1];
		force_.c = force[2];
		force_.d = force[3];
		force_.e = force[4];
		force_.f = force[5];
		pub1_.publish(force_);

    Eigen::Matrix<float, 5, 1> tor;
    tor = WPMIDtor.calculateID(q_pos_, v0, w0, dtime);
    aerial_manipulators::float5 tordif;
    tordif.a = tauki_[0] - tor(0, 0);
    tordif.b = tauki_[1] - tor(1, 0);
    tordif.c = tauki_[2] - tor(2, 0);
    tordif.d = tauki_[3] - tor(3, 0);
    tordif.e = tauki_[4] - tor(4, 0);
    pub2_.publish(tordif);

    aerial_manipulators::float5 torq;
    torq.a = tor(0, 0);
    torq.b = tor(1, 0);
    torq.c = tor(2, 0);
    torq.d = tor(3, 0);
    torq.e = tor(4, 0);
    pub3_.publish(torq);

		loop_rate.sleep();
		++count;
	}

	return;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	WPManipulatorForceEstimation WPMFE_node;
	WPMFE_node.run();
	return 0;
}
