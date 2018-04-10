/*
 *  Copyright (C) 2017, 2018 Ongun Kanat <ongun.kanat@gmail.com>
 *  Copyright (C) 2017       Arda İnceoğlu <93arda@gmail.com>
 *  Copyright (C) 2017, 2018 Istanbul Technical University
 *                           Artificial Intelligence and Robotics Laboratory
 *                           <air.cs.itu.edu.tr>
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
  * @file util.cpp
  * @authors Ongun Kanat <ongun.kanat@gmail.com>
  * @authors Arda İnceoğlu <93arda@gmail.com>
  * @brief Utilty functions/data structures implementation
  */

#include <violet/util.h>

namespace violet {
namespace util {

double volumeIntersectionRate( tf::Point existingLocation, tf::Point existingSize, tf::Point newLocation, tf::Point newSize)
{

    tf::Point existingSP; // start points
    tf::Point newSP;
    tf::Point existingEP; // end points
    tf::Point newEP;

    existingSP = existingLocation - (existingSize / 2);
    newSP = newLocation - (newSize / 2);
    existingEP = existingLocation + (existingSize / 2);
    newEP = newLocation + (newSize / 2);

    if(!checkRangesOverlap(existingSP.x(), existingEP.x(), newSP.x(), newEP.x()))
        return 0.0;

    double rateX = rangeIntersectionRate(existingSP.x(), existingEP.x(), newSP.x(), newEP.x());

    if(!checkRangesOverlap(existingSP.y(), existingEP.y(), newSP.y(), newEP.y()))
        return 0.0;

    double rateY = rangeIntersectionRate(existingSP.y(), existingEP.y(), newSP.y(), newEP.y());

    if(!checkRangesOverlap(existingSP.z(), existingEP.z(), newSP.z(), newEP.z()))
        return 0.0;

    double rateZ = rangeIntersectionRate(existingSP.z(), existingEP.z(), newSP.z(), newEP.z());

    return (rateX * rateY * rateZ);
}


void updateGaussianPDF(GaussianPdf &destination, GaussianPdf observation)
{
    Eigen::Matrix3d K = destination.sigma * (destination.sigma + observation.sigma).inverse();

    GaussianPdf new_pdf;
    new_pdf.mu = destination.mu + K * (observation.mu - destination.mu);
    new_pdf.sigma = destination.sigma - K * destination.sigma;

    // Convergence???

    //NaN
    if(new_pdf.mu[0] != new_pdf.mu[0] ||
       new_pdf.mu[1] != new_pdf.mu[1] ||
       new_pdf.mu[2] != new_pdf.mu[2]) {
        ROS_WARN("NaN location detected!");
        return;
    }

    destination = new_pdf;
}

} /* END OF NAMESPACE util */
} /* END OF NAMESPACE violet */
