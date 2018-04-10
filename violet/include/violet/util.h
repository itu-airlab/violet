/*
 *  Copyright (C) 2017 Ongun Kanat <ongun.kanat@gmail.com>
 *  Copyright (C) 2017 Arda İnceoğlu <93arda@gmail.com>
 *  Copyright (C) 2017 Istanbul Technical University
 *                     Artificial Intelligence and Robotics Laboratory
 *                     <air.cs.itu.edu.tr>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
  * @file util.h
  * @authors Ongun Kanat <ongun.kanat@gmail.com>
  * @authors Arda İnceoğlu <93arda@gmail.com>
  * @brief Utilty functions/data structures declarations and implementations for inline ones
  */
#ifndef UTIL_H
#define UTIL_H

#include "enums.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace violet {
/**
 * @brief Various functions without a common category
 */
namespace util {

/**
 * @brief Check if two objects overlap
 * @param existingLocation Location of the existing object
 * @param existingSize Size of the existing object
 * @param newLocation Location of the new object
 * @param newSize Size of the new object
 * @return true if the objects overlap in all 3 axes, false otherwise
 */
double volumeIntersectionRate(tf::Point existingLocation, tf::Point existingSize, tf::Point newLocation, tf::Point newSize );

/**
 * @brief Checks whether one of two number ranges contains the other one (inclusive)
 * @param range1_start The start point of the first range
 * @param range1_end The end point of the first range
 * @param range2_start The start point of the second rage
 * @param range2_end The end point of the second range
 * @param allowance The allowance value for overflows (0.5 cm default)
 * @return true , if the one of the ranges contains other one
 */
static inline bool checkRangesOverlap(double range1_start, double range1_end, double range2_start, double range2_end, double allowance = 0.01)
{
    return (range2_start <= range1_end + allowance) && (range1_start <= range2_end + allowance);
}

/**
 * @brief Returns the intersection rate of two ranges
 * @param range1_start The start point of the first range
 * @param range1_end The end point of the second range
 * @param range2_start The start point of the second range
 * @param range2_end The end point of the second range
 * @param threshold The threshold value which the intersection rate will be checked against
 * @return The intersection rate of range1 and range2
 */
static inline double rangeIntersectionRate(double range1_start, double range1_end, double range2_start, double range2_end)
{
    double intersection = std::min(range1_end, range2_end) - std::max(range1_start, range2_start);
    double union_size   = std::max(range1_end, range2_end) - std::min(range1_start, range2_start);
    return (intersection / union_size);
}

/**
 * @brief Multivariate normal distribution class
 */
struct GaussianPdf
{
    GaussianPdf()
    {}

    GaussianPdf(Eigen::Vector3d _mu, Eigen::Matrix3d _sigma) : mu(_mu), sigma(_sigma)
    {}

    /**
     * @brief mean
     */
    Eigen::Vector3d mu;
    /**
     * @brief covariance matrix
     */
    Eigen::Matrix3d sigma;
};

/**
 * @brief Merge the observation and destination distributions and write the result into destination
 * @param destination Distribution of destination
 * @param observation Distribution of observation
 */
void updateGaussianPDF(GaussianPdf &destination, GaussianPdf observation);



} /* END OF NAMESPACE util */
} /* END NAMESPACE violet */


#endif // UTIL_H

