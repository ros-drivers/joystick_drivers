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

#include "wiimote/stat_vector_3d.hpp"


#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <stdexcept>

StatVector3d::StatVector3d() {count_ = 0;}

StatVector3d::StatVector3d(int x, int y, int z)
{
  count_ = 0;

  addData(x, y, z);
}

void StatVector3d::clear()
{
  x_.clear();
  y_.clear();
  z_.clear();

  count_ = 0;
}

int StatVector3d::size() {return count_;}

void StatVector3d::addData(int x, int y, int z)
{
  ++count_;

  x_.push_back(x);
  y_.push_back(y);
  z_.push_back(z);
}

TVectorDouble StatVector3d::getMeanRaw()
{
  TVectorDouble result;

  if (x_.size() < 1) {
    throw std::runtime_error("Not enough data points for calculations");
  }

  double x_sum = std::accumulate(std::begin(x_), std::end(x_), 0.0);
  result.push_back(x_sum / x_.size());

  double y_sum = std::accumulate(std::begin(y_), std::end(y_), 0.0);
  result.push_back(y_sum / y_.size());

  double z_sum = std::accumulate(std::begin(z_), std::end(z_), 0.0);
  result.push_back(z_sum / z_.size());

  return result;
}

TVectorDouble StatVector3d::getMeanScaled(double scale)
{
  TVectorDouble mean = getMeanRaw();

  std::transform(
    mean.begin(), mean.end(), mean.begin(), std::bind1st(std::multiplies<double>(), scale));

  return mean;
}

TVectorDouble StatVector3d::getVarianceRaw()
{
  TVectorDouble result;

  TVectorDouble mean = getMeanRaw();

  if (x_.size() < 2) {
    throw std::runtime_error("Not enough data points for calculations");
  }

  double accum = 0.0;
  std::for_each(
    std::begin(x_), std::end(x_),
    [&](const double d)
    {accum += (d - mean.at(0)) * (d - mean.at(0));});

  result.push_back(accum / (x_.size() - 1));

  accum = 0.0;
  std::for_each(
    std::begin(y_), std::end(y_),
    [&](const double d)
    {accum += (d - mean.at(1)) * (d - mean.at(1));});

  result.push_back(accum / (y_.size() - 1));

  accum = 0.0;
  std::for_each(
    std::begin(z_), std::end(z_),
    [&](const double d)
    {accum += (d - mean.at(2)) * (d - mean.at(2));});

  result.push_back(accum / (z_.size() - 1));

  return result;
}

TVectorDouble StatVector3d::getVarianceScaled(double scale)
{
  TVectorDouble variance = getVarianceRaw();

  std::transform(
    variance.begin(), variance.end(), variance.begin(),
    std::bind1st(std::multiplies<double>(), scale));

  return variance;
}

TVectorDouble StatVector3d::getStandardDeviationRaw()
{
  TVectorDouble result;

  TVectorDouble variance = getVarianceRaw();

  result.push_back(sqrt(variance.at(0)));
  result.push_back(sqrt(variance.at(1)));
  result.push_back(sqrt(variance.at(2)));

  return result;
}

TVectorDouble StatVector3d::getStandardDeviationScaled(double scale)
{
  TVectorDouble stddev = getStandardDeviationRaw();

  std::transform(
    stddev.begin(), stddev.end(), stddev.begin(), std::bind1st(std::multiplies<double>(), scale));

  return stddev;
}
