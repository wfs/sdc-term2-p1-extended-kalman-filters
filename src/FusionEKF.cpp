#include "FusionEKF.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::endl;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);  // laser sensor measurement covariance matrix
    R_radar_ = MatrixXd(3, 3);  // radar sensor measurement covariance matrix
    H_laser_ = MatrixXd(2, 4);  // laser sensor measurement matrix
    Hj_ = MatrixXd(3, 4);       // radar sensor Jacobian matrix

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    // Laser measurement matrix
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    ekf_.x_ = VectorXd(4);    // object state
    ekf_.P_ = MatrixXd(4, 4);  // object covariance matrix
    // easier to update F matrix from its identity matrix form
    ekf_.F_ = MatrixXd::Identity(4, 4);  // state transition matrix
    //ekf_.F_(0,2) = 1;
    //ekf_.F_(1,3) = 1;
    ekf_.Q_ = MatrixXd(4, 4);  // process covariance matrix
    //measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             * Convert radar from polar to cartesian coordinates and initialize state.
             * See Polar <-> Cartesian Coordinates - http://www.mathsisfun.com/polar-cartesian-coordinates.html
            */
            double px_cart = measurement_pack.raw_measurements_[0] *
                             cos(measurement_pack.raw_measurements_[1]);  // rho * cos(phi)
            double py_cart = measurement_pack.raw_measurements_[0] *
                             sin(measurement_pack.raw_measurements_[1]);  // rho * sin(phi)
            double vx_cart = measurement_pack.raw_measurements_[2] *
                             cos(measurement_pack.raw_measurements_[1]);  // rho_dot * cos(phi)
            double vy_cart = measurement_pack.raw_measurements_[2] *
                             sin(measurement_pack.raw_measurements_[1]);  // rho_dot * sin(phi)

            //Initialize the state ekf_.x_ with the first measurement
            //ekf_.x_ << x_cart, y_cart, 0, 0;
            ekf_.x_ << px_cart, py_cart, vx_cart, vy_cart;


        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state with 1st location and 0 velocity.
            */
            double px = measurement_pack.raw_measurements_[0];  // px : x location
            double py = measurement_pack.raw_measurements_[1];  // py : y location
            //Initialize the state ekf_.x_ with the first measurement
            ekf_.x_ << px, py, 0, 0;  // zero velocity
        }
        // The first measurement is reliable so use the
        // identity matrix as the initial covariance matrix
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << MatrixXd::Identity(4, 4);
        //cout << "EKF initial state matrix is: "  << endl;
        //cout << ekf_.x_  << endl;

        //cout << "EKF initial covariance matrix is: "  << endl;
        //cout << ekf_.P_  << endl;

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;

        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    //cout << measurement_pack.raw_measurements_ << endl;
    //cout << "previous_timestamp_ is:  "<< previous_timestamp_<<endl;
    // divide dt to get seconds units
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // lapsed time in secs
    previous_timestamp_ = measurement_pack.timestamp_;

    // lapsed time power calcs e.g. dt_4 == dt^4
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    //cout << "measurement_pack.timestamp_ is:  "<< measurement_pack.timestamp_<<endl;
    // update F matrix to include the delta time, dt
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;


    //set the process covariance matrix Q
    double noise_ax = 9;   // x uncertainty added
    double noise_ay = 9;   // y uncertainty added

    // Update the process covariance matrix, Q to include time delta
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, R_laser_, ekf_.Q_);
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    //cout << "x_ = " << ekf_.x_ << endl;
    //cout << "P_ = " << ekf_.P_ << endl;
}
