/*
 * Algorithm.hpp
 *
 *  Created on: Mar 28, 2017
 *      Author: hradt
 */

#ifndef ALGORITHM_HPP_
#define ALGORITHM_HPP_

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
namespace husky_highlevel_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  void updateData(const double minRange, const double minRangeAngle);

  /*!
   * Get the computed average of the data.
   * @return the average of the data.
   */
  double getMinRange() const;
  double getMinRangeAngle() const;

 private:

  double minRange_;
  double minRangeAngle_;


};

} /* namespace */



#endif /* ALGORITHM_HPP_ */
