// MIT License

// Copyright (c) 2022 BobMcFry

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// source: 
// https://github.com/BobMcFry/averaging_weighted_quaternions


#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>

tf::Quaternion getAverageQuaternion(
  const std::vector<tf::Quaternion>& quaternions, 
  const std::vector<double>& weights)
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
  Eigen::Vector3d vec;
  for (size_t i = 0; i < quaternions.size(); ++i)
  {
    // Weigh the quaternions according to their associated weight
    tf::Quaternion quat = quaternions[i] * weights[i];
    // Append the weighted Quaternion to a matrix Q.
    Q(0,i) = quat.x();
    Q(1,i) = quat.y();
    Q(2,i) = quat.z();
    Q(3,i) = quat.w();
  }

  // Create a solver for finding the eigenvectors and eigenvalues
  Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

  // Find index of maximum (real) Eigenvalue.
  auto eigenvalues = es.eigenvalues();
  size_t max_idx = 0;
  double max_value = eigenvalues[max_idx].real();
  for (size_t i = 1; i < 4; ++i)
  {
    double real = eigenvalues[i].real();
    if (real > max_value)
    {
      max_value = real;
      max_idx = i;
    }
  }

  // Get corresponding Eigenvector, normalize it and return it as the average quat
  auto eigenvector = es.eigenvectors().col(max_idx).normalized();

  tf::Quaternion mean_orientation(
    eigenvector[0].real(),
    eigenvector[1].real(),
    eigenvector[2].real(),
    eigenvector[3].real()
  );

  return mean_orientation;
}