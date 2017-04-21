// Comment / Uncomment lines in CMakeLists.txt that refer to *.cpp with main() functions
// then Uncomment next line
//#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do this in one cpp file.
//                          Once you have more than one file with unit tests in you'll just #include "catch.hpp" and go.
//                          https://github.com/philsquared/Catch/blob/master/docs/tutorial.md
#include "catch.hpp"
#include "tools.cpp"

using namespace std;

/**
 * RMSE accuracy calc of Estimation vector VS Ground Truth vector returns [0.1, 0.1, 0.1, 0,1].
 */

SCENARIO("RMSE accuracy calc of Estimation vector VS Ground Truth vector returns [0.1, 0.1, 0.1, 0,1].",
         "[rmse_calc]") {

    GIVEN("Lists of 4 estimations and corresponding ground truth values") {
        vector<VectorXd> estimations;
        vector<VectorXd> ground_truth;

        //the input list of estimations
        VectorXd e(4);
        e << 1, 1, 0.2, 0.1;
        estimations.push_back(e);
        e << 2, 2, 0.3, 0.2;
        estimations.push_back(e);
        e << 3, 3, 0.4, 0.3;
        estimations.push_back(e);

        //the corresponding list of ground truth values
        VectorXd g(4);
        g << 1.1, 1.1, 0.3, 0.2;
        ground_truth.push_back(g);
        g << 2.1, 2.1, 0.4, 0.3;
        ground_truth.push_back(g);
        g << 3.1, 3.1, 0.5, 0.4;
        ground_truth.push_back(g);

        REQUIRE(estimations.size() == ground_truth.size()); // Vector rows length needs to be same

        WHEN("we Calculate RMSE") {
            Tools tools;
            VectorXd result = tools.CalculateRMSE(estimations, ground_truth);

            THEN("results in low error / high estimation accuracy values of 0.1") {
                float r = result(0);  // 0.1
                float t = 0.1;  // test

                REQUIRE(r == t);
                REQUIRE(result.size() == 4);
                //cout << result.size() << endl;
                //cout << result << endl;
            }
        }
    }
}

/**
 * Calc Jacobian values Hj based on x_predicted = 4, returning 3x4 matrix with Hj(0)(0) == 0.447214.
 */

SCENARIO("Calc Jacobian values Hj based on 4x1 x_predicted vector, returning 3x4 matrix with element Hj(0)(0) = 0.447214.",
         "[jacobian_calc]") {

    GIVEN("x_predicted vector contains 4 object state values ; Px, Py, Vx, Vy") {
        VectorXd x_predicted(4);
        x_predicted << 1, 2, 0.2, 0.4;

        REQUIRE(x_predicted.size() == 4);
        REQUIRE(x_predicted(0) == 1);
        //cout << x_predicted.size() << endl;
        //cout << x_predicted << endl;

        WHEN("we Calculate Jacobian") {
            Tools tools;
            MatrixXd result = tools.CalculateJacobian(x_predicted);

            THEN("a 3x4 matrix with Jacobian values is returned") {
                string r = to_string(result(0,0));
                string t = "0.447214";  // test

                REQUIRE(r == t);
                REQUIRE(result.size() == 12);
                //cout << result.size() << endl;
                //cout << result << endl;
            }
        }
    }
}