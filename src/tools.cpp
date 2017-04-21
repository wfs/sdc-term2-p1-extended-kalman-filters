#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }
    //cout << "size of estimations.size() " << estimations.size()<< endl;
    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];
        //cout << "residual = estimations[i] - ground_truth[i]   "<< endl << residual << endl;
        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();


    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */

    /**
     * z-state : 3 rows of measurement ; Range 'rho', Bearing 'phi', Range Rate 'rho dot'.
     * x-state : 4 cols of prediction ; Position Px, Py & Velocity Vx, Vy.
     */
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    /**
     * c3 = c1^1 * c2^0.5 = c3^1.5 <-> c3^3/2
     * See algebra Laws of Exponents - http://www.mathsisfun.com/algebra/exponent-laws.html
     */
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    //check division by zero
    if (fabs(c1) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
            -(py / c1), (px / c1), 0, 0,
            py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
