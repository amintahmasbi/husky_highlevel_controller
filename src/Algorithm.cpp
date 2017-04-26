/*
 * Algorithm.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: hradt
 */

#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller {

Algorithm::Algorithm()
    : minRange_(0.0),
      minRangeAngle_(0.0)
{
}

Algorithm::~Algorithm()
{
}

void Algorithm::updateData(const double minRange, const double minRangeAngle)
{
	minRange_ = minRange;
	minRangeAngle_ = minRangeAngle;
}

double Algorithm::getMinRange() const
{
  return minRange_;
}

double Algorithm::getMinRangeAngle() const
{
  return minRangeAngle_;
}

} /* namespace */
