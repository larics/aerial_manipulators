/**
 * @class KalmanFilter
 * @brief An implementation of the Kalman filter.
 *
 * The Kalman filter is an efficient recursive filter that estimates the internal state of a
 * linear dynamic system from a series of noisy measurements. This means that only the estimated
 * state from the previous time step and the current measurement are needed to compute the estimate
 * for the current state. It is conceptualized as two distinct phases: "predict" or "model update"
 * and "correction" or "measure update". The predict phase uses the state estimate from the previous
 * timestep to produce an estimate of the state at the current timestep. This predicted state estimate
 * is also known as the a priori state estimate because, although it is an estimate of the state at the
 * current timestep, it does not include observation information from the current timestep. In the
 * correction phase, the current a priori prediction is combined with current observation information
 * to refine the state estimate. This improved estimate is termed the a posteriori state estimate. Typically,
 * the two phases alternate, with the prediction advancing the state until the next scheduled observation,
 * and the correction incorporating the observation. If an observation is unavailable for some reason,
 * the correction may be skipped and multiple prediction steps performed. Likewise, if multiple independent
 * observations are available at the same time, multiple correction steps may be performed.
 *
 * @details
 * Contact: carek.marko@gmail.com
 *
 * @author Marko Car
 * @version 0.1
 * @date May 2018
 *
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H


class KalmanFilter {
    public:
        /**
         * @brief A constructor.
         * Initializes all private variables.
         */
        KalmanFilter(void);

        /**
         * @brief Method for prediction. Updates the states using simplified model (integratior)
         *
         * @param dt Time from last update
         */
        void modelUpdate(double dt);

        /**
         * @brief Corrects the states using new measurement.
         *
         * @param pos_m New measurement
         */
        void measureUpdate(double pos_m);

        /**
         * @brief Method that returns estimated velocity
         *
         * @return Estimated velocity
         */
        double getVelocity(void);

        /**
         * @brief Method that returns estimated position
         *
         * @return Estimated position
         */
        double getPosition(void);

        /**
         * @brief Sets the kalman filter parameter measurement noise
         *
         * @param r Measurement noise
         */
        void setMeasureNoise(double r);

        /**
         * @brief Method sets the kalman filter process noise (position)
         *
         * @param q Process noise (position)
         */
        void setPositionNoise(double q);

        /**
         * @brief Method sets the kalman filter process noise (velocity)
         *
         * @param q Process noise (velocity)
         */
        void setVelocityNoise(double q);

        /**
         * @brief Method sets initial position value
         *
         * @param pos Position value
         */
        void initializePosition(double pos);

    private:
        double x_cov_[2][2], x_[2];
        double q_[2], r_;
};


#endif //DETECTION_KALMANFILTER_H
