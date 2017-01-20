/*
 * Three dimensional statistic vector for use with the
 * for ROS Node which interfaces with a wiimote control unit.
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * Initial C++ implementation by
 *   Mark Horn <mark.d.horn@intel.com>
 *
 * Revisions:
 *
 */

#pragma once
#ifndef WIIMOTE_STAT_VECTOR_3D_H
#define WIIMOTE_STAT_VECTOR_3D_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>

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

#endif  // WIIMOTE_STAT_VECTOR_3D_H
