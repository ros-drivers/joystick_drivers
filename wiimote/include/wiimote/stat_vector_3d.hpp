// Copyright 2020 Intel Corporation
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/*
 * Three dimensional statistic vector for use with the
 * for ROS Node which interfaces with a wiimote control unit.
 */

/*
 * Initial C++ implementation by
 *   Mark Horn <mark.d.horn@intel.com>
 *
 * Revisions:
 *
 */

#pragma once
#ifndef WIIMOTE__STAT_VECTOR_3D_HPP_
#define WIIMOTE__STAT_VECTOR_3D_HPP_

#include <vector>

// The vector of 3 values collected to generate:
// mean, standard deviation, and variance.

typedef std::vector<double> TVectorDouble;

class StatVector3d
{
public:
  StatVector3d();
  StatVector3d(int x, int y, int z);

  void clear();

  int size();
  void addData(int x, int y, int z);

  TVectorDouble getMeanRaw();
  TVectorDouble getMeanScaled(double scale);
  TVectorDouble getVarianceRaw();
  TVectorDouble getVarianceScaled(double scale);
  TVectorDouble getStandardDeviationRaw();
  TVectorDouble getStandardDeviationScaled(double scale);

private:
  int count_;
  std::vector<int> x_;
  std::vector<int> y_;
  std::vector<int> z_;
};

#endif  // WIIMOTE__STAT_VECTOR_3D_HPP_
