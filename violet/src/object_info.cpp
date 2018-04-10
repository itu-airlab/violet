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
 * @file object_info.cpp
 * @author Arda İnceoğlu <93arda@gmail.com>
 * @author Ongun Kanat <ongun.kanat@gmail.com>
 * @brief Implementation of the ObjectInfo class
 */
#include <violet/object_info.h>
#include <violet/object_catalog.h>

namespace violet {
ObjectInfo::ObjectInfo() /* TODO: add object catalog as a paramter to constructor */
{
    _probability = 0.5;
    bayesian_fusion::initializePosteriorValues(attributes, object_catalog::cataloged_objects);
    for(bayesian_fusion::PosteriorMap::iterator it = attributes.begin(); it != attributes.end(); ++it) {
        attribute_heap.push_back(&(it->second));
    }
    std::make_heap(attribute_heap.begin(), attribute_heap.end(), attribute_cell_comparator);
#ifdef CLOSED_WORLD_KB
    _recognition_threshold_exceeded = false;
#endif
}

double ObjectInfo::probability()
{
    return _probability;
}

void ObjectInfo::setProbability(double prob)
{
    this->_probability = prob;
}


void ObjectInfo::increaseConfidence(const boost::array<boost::array<double, 2>, 2> confidence_confusion)
{
    if(_probability > 0.999)
        return;

    double next_probability = confidence_confusion[0][0] * _probability;
    double negative = confidence_confusion[1][0] * (1 -  _probability);
    _probability = next_probability / (next_probability + negative);
}

void ObjectInfo::decreaseConfidence(const boost::array<boost::array<double, 2>, 2> confidence_confusion)
{
    double next_negative = confidence_confusion[1][1] * (1 - _probability);
    double next_positive = confidence_confusion[0][1] * _probability;
    _probability = 1 - (next_negative / (next_negative + next_positive));
}

void ObjectInfo::updateAttributes(const bayesian_fusion::ConfusionMatrix &confusion_matrix, const std::string &observation)
{
    const ObjectDatabaseEntry* previous_object = attribute_heap[0]->object;

    bayesian_fusion::updatePosteriorValues(attributes, confusion_matrix, observation);
    std::make_heap(attribute_heap.begin(), attribute_heap.end(), attribute_cell_comparator);

    const ObjectDatabaseEntry* object_after_update = attribute_heap[0]->object;

    if(previous_object != object_after_update) {
        _size = object_after_update->size;
    }

#ifdef CLOSED_WORLD_KB
    /* TODO: Make this threshold parametrized */
    if(!_recognition_threshold_exceeded)
        _recognition_threshold_exceeded = attribute_heap[0]->probability > 0.9;
#endif
}

std::pair<std::string, double> ObjectInfo::attribute(ObjectAttribute_T _attribute) const
{
    const bayesian_fusion::PosteriorCell* most_probable = attribute_heap[0];
    if(most_probable->probability < 0.9) {
        return std::make_pair(std::string(""), 0.0);
    }
    std::pair<std::string, double> result;
    switch(_attribute) {
        case O_ATTR_COLOR:
            result.first = most_probable->object->color;
        break;

        case O_ATTR_MATERIAL:
            result.first = most_probable->object->material;
        break;

        case O_ATTR_SHAPE:
            result.first = most_probable->object->shape;
        break;

        case O_ATTR_TYPE:
            result.first = most_probable->object->type;
        break;

        case O_ATTR_NAME:
            result.first = most_probable->object->name;
        break;
    }

    result.second = most_probable->probability;
    return result;
}


void ObjectInfo::setLastDetection(std::string source_name, ros::Time _time)
{
    last_detections[source_name] = _time;
}

ros::Time ObjectInfo::lastDetectionTime(std::string source_name) const
{
    std::map<std::string, ros::Time>::const_iterator ld = last_detections.find(source_name);
    if( ld != last_detections.end() ) {
        return ld->second;
    }

    return ros::Time(0);
}

void ObjectInfo::setLocation(tf::Point new_location)
{
    _location = new_location;
}

tf::Point ObjectInfo::location() const
{
    return _location;
}

void ObjectInfo::setSize(tf::Point new_size)
{
    _size = new_size;
}

tf::Point ObjectInfo::size() const
{
    return _size;
}

tf::Quaternion ObjectInfo::orientation() const
{
    return _orientation;
}

void ObjectInfo::setOrientation(tf::Quaternion new_orientation)
{
    _orientation = new_orientation;
}

util::GaussianPdf ObjectInfo::locationPDF() const
{
    return _location_pdf;
}

void ObjectInfo::setLocationPDF(util::GaussianPdf pdf)
{
    _location_pdf = pdf;
}

std::string ObjectInfo::fovFrame() const
{
    return _fov_frame;
}

void ObjectInfo::setFovFrame(std::string frame_name)
{
    _fov_frame = frame_name;
}

#ifdef CLOSED_WORLD_KB
void ObjectInfo::mergeAttributes(const ObjectInfo &other_object)
{
    const bayesian_fusion::PosteriorMap& other_attr = other_object.attributes;
    for(bayesian_fusion::PosteriorMap::iterator attr_it = attributes.begin(); attr_it != attributes.end(); ++attr_it) {
        bayesian_fusion::PosteriorMap::const_iterator other_cell = other_attr.find(attr_it->first);

        ROS_ASSERT_MSG(other_cell != other_attr.end(),
                       "Error in %s: Can't merge attributes of objects with different sets of posteriors", __PRETTY_FUNCTION__);

        attr_it->second.probability = (attr_it->second.probability + other_cell->second.probability) * 0.5;
    }
    std::make_heap(attribute_heap.begin(), attribute_heap.end(), attribute_cell_comparator);
}
#endif


} /* END NAMESPACE violet */

