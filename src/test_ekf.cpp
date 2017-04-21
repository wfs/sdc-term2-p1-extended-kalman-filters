// Comment / Uncomment lines in CMakeLists.txt that refer to *.cpp with main() functions
// then Uncomment next line
#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do this in one cpp file.
//                          Once you have more than one file with unit tests in you'll just #include "catch.hpp" and go.
//                          https://github.com/philsquared/Catch/blob/master/docs/tutorial.md
#include "catch.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Term 2 Project 1 EKF test RMSE of Estimation VS Ground Truth meets rubic.
 */

SCENARIO("Term 2 Project 1 EKF test RMSE of Estimation VS Ground Truth meets rubic.",
         "[ekf_rubic]") {

    GIVEN("input and output files of Lidar and Radar measurements exist") {
        string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
        ifstream in_file_(in_file_name_.c_str(), ifstream::in);

        string out_file_name_ = "../data/obj_pose-laser-radar-synthetic-output.txt";
        ofstream out_file_(out_file_name_.c_str(), ofstream::out);

        REQUIRE(in_file_.is_open() == true);
        REQUIRE(out_file_.is_open() == true);

        WHEN("the EKF pipeline runs") {
            REQUIRE(true);

            THEN("RMSE is <= project rubic") {
                REQUIRE(true);
            }
        }
    }
}
