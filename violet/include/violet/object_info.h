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
 * @file object_info.h
 * @author Arda İnceoğlu <93arda@gmail.com>
 * @author Ongun Kanat <ongun.kanat@gmail.com>
 * @brief ObjectInfo class declaration
 */
#ifndef OBJECTINFO_H_
#define OBJECTINFO_H_

/* Standard Libs */
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <limits>
#include <map>
#include <algorithm>
#include <queue>
/* ROS */
#include <ros/ros.h>
#include <tf/tf.h>
/* Package's headers */
#include "enums.h"
#include "util.h"
#include "bayesian_fusion.h"

namespace violet {
/**
 * @brief Container and methods for object information stored in the KB.
 */
class ObjectInfo
{
public:
    static struct
    {
        typedef const bayesian_fusion::PosteriorCell* CellPtr;
        bool operator()(const CellPtr &first, const CellPtr &second) const
        {
            return first->probability < second->probability;
        }
    } attribute_cell_comparator;

private:
    /**
     * @brief probability that the object exists in this location
     */
    double _probability;

    /**
     * @brief size of the matched template
     */
    tf::Point _size;
    /**
     * @brief 3D location of center point of the matched template
     */
    tf::Point _location;
    /**
     * @brief 3D Orientation as a quaternion of the matched template
     */
    tf::Quaternion _orientation;
    /**
     * @brief Normal distribution as the uncertainity of location of the of the object
     */
    util::GaussianPdf _location_pdf;
    /**
     * @brief Map of the last detection times of this object by each source
     */
    std::map <std::string, ros::Time> last_detections;

    /**
     * @brief The attribute list of this object
     */
    bayesian_fusion::PosteriorMap attributes;

    /**
     * @brief A heap that holds references to attribute values, provides efficient access
     */
    std::vector<const bayesian_fusion::PosteriorCell*> attribute_heap;

    /**
     * @brief Field of view frame of this object
     */
    std::string _fov_frame;

#ifdef CLOSED_WORLD_KB
    bool _recognition_threshold_exceeded;
#endif

public:

    /**
     * @brief Default constructor
     */
    ObjectInfo();

    /**
     * @brief Get probability
     * @return Probability value of this object
     */
    double probability();
    /**
     * @brief Sets the probability
     * @param prob The probability value
     */
    void setProbability(double prob);

    /**
     * @brief Increase confidence value as the Bayesian probability measure. This will increase
     * the probability of the ObjectInfo instance.
     * @param confidence_confusion A matrix that denotes the confidence confusion of the update
     */
    void increaseConfidence(const boost::array<boost::array<double, 2>, 2> confidence_confusion);
    /**
     * @brief Decrase confidence value as the Bayesian probability measure. This will decrease
     * the probability of the ObjectInfo instance.
     * @param source_name The name of the input source
     */
    void decreaseConfidence(const boost::array<boost::array<double, 2>, 2> confidence_confusion);

    /**
     * @brief Adds a value to a specific attribute of an object and updates frequency heap accordingly
     * @param _attribute The atrribute
     * @param _value A value and its probabability
     */
    void updateAttributes(const bayesian_fusion::ConfusionMatrix &confusion_matrix, const std::string &observation);

    /**
     * @brief Returns the most probable detection for an attribute
     * @param _attribute The attribute whose most probable detection is being wanted
     * @return The pair of the probability of a detection and its value
     */
    std::pair<std::string, double> attribute(ObjectAttribute_T _attribute) const;

    /**
     * @brief Sets last detection time of this object by input source source_name
     * @param source_name Source name (e.g. "LineMod")
     * @param _time Last detection time
     */
    void setLastDetection(std::string source_name, ros::Time _time);

    /**
     * @brief Returns last detection time of this object by input source source_name
     * @param source_name Source name (e.g. "LineMod")
     * @return Last detection time
     */
    ros::Time lastDetectionTime(std::string source_name) const;

    /**
     * @brief Set current location of this object
     * @param new_location Location
     */
    void setLocation(tf::Point new_location);
    /**
     * @brief Get current location
     * @return The current location as tf::Point
     */
    tf::Point location() const;

    /**
     * @brief Set size of the object
     * @param new_size The new size of the object
     */
    void setSize(tf::Point new_size);
    /**
     * @brief Returns current size of the object
     * @return The current size as tf::Point
     */
    tf::Point size() const;

    /**
     * @brief Returns current orientation of the object
     * @return The current orientation as tf::Quaternion
     */
    tf::Quaternion orientation() const;
    /**
     * @brief Sets current orientation of the object
     * @param new_orientation The new orientation
     */
    void setOrientation(tf::Quaternion new_orientation);

    util::GaussianPdf locationPDF() const;
    void setLocationPDF(util::GaussianPdf pdf);

    std::string fovFrame() const;
    void setFovFrame(std::string frame_name);

#ifdef CLOSED_WORLD_KB
    bool isRecognitionThresholdExceeded() const { return _recognition_threshold_exceeded; }
    void mergeAttributes(const ObjectInfo &other_object);
#endif

};
} /* END OF NAMESPACE violet */

#endif /* OBJECTINFO_H_ */

