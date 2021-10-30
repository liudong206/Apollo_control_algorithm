/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <limits>

//#include "eigen3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Dense"

//#include "modules/common/log.h"
#include "linear_quadratic_regulator.h"
#include <iostream>

//namespace apollo {
//namespace common {
namespace math {

using Matrix = Eigen::MatrixXd;

void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
//    AERROR << "LQR solver: one or more matrices have incompatible dimensions.";
    std::cout<< "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
    return;
  }

  Matrix AT = A.transpose();
  Matrix BT = B.transpose();

  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Matrix P = Q;
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Matrix P_next =
        AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  if (num_iteration >= max_num_iteration) {
//    AWARN << "LQR solver cannot converge to a solution, "
//             "last consecutive result diff. is:"
//          << diff;
  std::cout<< "LQR solver cannot converge to a solution" << std::endl;
  std::cout<< "last consecutive result diff. is:" << diff << std::endl;
  } else {
//    ADEBUG << "LQR solver converged at iteration: " << num_iteration
//           << ", max consecutive result diff.: " << diff;
//  std::cout<< "LQR solver converged at iteration:" << std::endl;
//  std::cout<< "max consecutive result diff. : " << diff << std::endl;
  }
  *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

}  // namespace math
//}  // namespace common
//}  // namespace apollo
