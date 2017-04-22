#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;  // object state
    P_ = P_in;  // object covariance matrix
    F_ = F_in;  // state transition matrix
    H_ = H_in;  // measurement matrix
    R_ = R_in;  // measurement covariance matrix
    Q_ = Q_in;  // process covariance matrix
}

void KalmanFilter::Predict() {
    /**
    TODO:
      * predict the state
    */
    /**
    * KF Prediction step
    *
    * VectorXd x_' : predicted (object state) location Px, Py and velocity Vx, Vy
    * MatrixXd P_' : location covariance matrix aka uncertainty of predicted location, x'
    * MatrixXd F_ : next state transition function
    * VectorXd x_ : state aka where we think we are located ; Px, Py and velocity Vx, Vy
    * MatrixXd Q_ : 'u' is the process (prediction) noise (a gaussian with zero mean)
    *              with a covariance matrix 'Q' (aka the 'process covariance', a 2x2 zero matrix).
    * MatrixXd Ft : F transposed
    */
    x_ = F_ * x_;
    //x = F * x + u; // State prediction noise = 0
    //cout << " The predict x state is " << x_ << endl;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    //cout << " The covariance matrix, P, is "  <<  P_<< endl;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */

    /**
     * KF Measurement update step
     *
     * IMPORTANT :
     *      LIDAR measurement updates ; use the H matrix for calculating y, S, P => K
     *      RADAR measurement updates ; use jacobian Hj matrix for calculating S, P => K
     *
     * VectorXd y_ : lidar ; error calculation given the new measurement z
     * VectorXd y_radar : radar ; error calculation given the new measurement z_radar
     * VectorXd z_pred : lidar sensor measurements
     * VectorXd z_radar :
     *      Radar sensor outputs values in polar coordinates, not cartesian coordinates like lidar.
     *      We need to convert x' (from prediction step) to polar coordinates
     *      i.e. the function h(x) maps values from cartesian to polar coordinates.
     *      so radar error measurement equation becomes y = z_radar - h(x')
     * MatrixXd H : measurement function matrix
     * VectorXd x : initial state position (aka location) Px, Py and velocity Vx, Vy
     * MatrixXd Ht = H transposed
     * MatrixXd P : object position covariance matrix aka initial position uncertainty
     *              as standard deviations from mean
     * MatrixXd R : measurement covariance matrix aka measurement uncertainty of sensors
     * MatrixXd S : combines uncertainty of where we think we are, P
     *              with the uncertainty of our sensor measurement, R
     * MatrixXd K :
     *      if sensor measurement uncertainty is high (R > P), then
     *          greater weight given to where we think we are, x.
     *      if position uncertainty is high (P > R), then
     *          greater weight given to sensor measurement, z.
     */
    // z is the measured value of the sensors
    H_ = MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
            0, 1, 0, 0;
    //cout << "x_ is: " << x_ << std::endl;
    //cout << "H_ in laser is: " << H_ << std::endl;

    VectorXd z_pred = H_ * x_;
    VectorXd y_ = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S_ = H_ * PHt + R_;
    MatrixXd Si = S_.inverse();
    MatrixXd K_ = PHt * Si;

    //new estimate
    x_ = x_ + (K_ * y_);

    long long x_size = x_.size();  // Needed in 64-bit machines
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_) * P_;
    //cout << "P_ new estimate is: " << P_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */

    /**
     * Convert radar from polar to cartesian coordinates and initialize state.
     */
    double rho = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    //double phi = atan(x_[1] / x_[0]);  // non-normalised bearing.
    double phi = atan2((x_[1] + M_1_PI), x_[0]);  // normalise bearing 'phi' between -pi and pi.
    //double rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    double new_rho = rho;
    if (new_rho < 1e-6) new_rho = 1e-6;  // replace 0 values with .00001
    double rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / new_rho;

    //cout << "rho : " << rho << " phi : " << phi << " rho_dot : " << rho_dot << endl;
    //cout << " phi : " << phi << endl;
    //MatrixXd z_pred_radar(3, 1);
    VectorXd z_pred_radar(3);
    z_pred_radar << rho, phi, rho_dot;
    VectorXd y_ = z - z_pred_radar;

    // Normalising angels - https://discussions.udacity.com/t/rmse-value-to-high-for-new-data-file/241643/6?u=andrew22
    //while (y_[1] < -M_PI) y_[1] += 2 * M_PI;
    //while (y_[1] > M_PI) y_[1] -= 2 * M_PI;

    MatrixXd Ht = H_.transpose();

    MatrixXd S_ = H_ * P_ * Ht + R_;
    MatrixXd Si = S_.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K_ = PHt * Si;

    //new estimate
    x_ = x_ + (K_ * y_);
    //cout << "x_ laser new estimate is: " << std::endl;
    //cout << x_<< std::endl;
    long long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_) * P_;
}
