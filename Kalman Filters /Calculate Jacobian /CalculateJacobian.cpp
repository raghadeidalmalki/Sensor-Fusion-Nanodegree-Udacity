#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}


MatrixXd CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    
    // Recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Calculate the terms needed for Jacobian matrix
    float c1 = px * px + py * py;  // px^2 + py^2
    float c2 = sqrt(c1);            // sqrt(px^2 + py^2)
    float c3 = c1 * c2;             // (px^2 + py^2) * sqrt(px^2 + py^2)

    // Check division by zero
    if (fabs(c1) < 1e-4) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    // Compute the Jacobian matrix
    Hj << (px / c2),            (py / c2),            0,     0,
          -(py / c1),          (px / c1),            0,     0,
          py * (vx * py - vy * px) / c3,  px * (vy * px - vx * py) / c3,  (px / c2), (py / c2);

    return Hj;
}
