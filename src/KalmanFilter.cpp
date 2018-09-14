#include <aerial_manipulators/KalmanFilter.h>

KalmanFilter::KalmanFilter(void) {
    //Initialization of the covariances and
    //measure and model noises
    x_[0] = 0;
    x_[1] = 0;
    x_cov_[0][0] = 1;
    x_cov_[0][1] = 0;
    x_cov_[1][0] = 0;
    x_cov_[1][1] = 1;

    q_[0] = 1;
    q_[1] = 10;
    r_ = 10;
}

void KalmanFilter::initializePosition(double pos) {
    x_[0] = pos;
}

void KalmanFilter::setPositionNoise(double q) {
    q_[0] = q;
}

void KalmanFilter::setVelocityNoise(double q) {
    q_[1] = q;
}

void KalmanFilter::setMeasureNoise(double r) {
    r_ = r;
}

void KalmanFilter::modelUpdate(double dt) {
    x_[0] = x_[0] + dt * x_[1];
    x_cov_[0][0] = x_cov_[0][0] + dt * (x_cov_[1][0] + x_cov_[0][1]) + dt * dt * x_cov_[1][1] + q_[0];
    x_cov_[0][1] = x_cov_[0][1] + dt * x_cov_[1][1];
    x_cov_[1][0] = x_cov_[1][0] + dt * x_cov_[1][1];
    x_cov_[1][1] = x_cov_[1][1] + q_[1];
}

void KalmanFilter::measureUpdate(double pos_m) {
    double sk, k1, k2, dk;

    sk = x_cov_[0][0] + r_;
    k1 = x_cov_[0][0] / sk;
    k2 = x_cov_[1][0] / sk;
    dk = pos_m - x_[0];
    x_[0] = x_[0] + k1 * dk;
    x_[1] = x_[1] + k2 * dk;
    x_cov_[0][0] = (1 - k1) * x_cov_[0][0];
    x_cov_[0][1] = (1 - k1) * x_cov_[0][1];
    x_cov_[1][0] = -k2 * x_cov_[0][0] + x_cov_[1][0];
    x_cov_[1][1] = -k2 * x_cov_[0][1] + x_cov_[1][1];
}

double KalmanFilter::getPosition(void) {
    return x_[0];
}

double KalmanFilter::getVelocity(void) {
    return x_[1];
}
