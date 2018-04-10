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

#include <string>
#include <map>
#include <tf/transform_listener.h>
#include <violet/input_source_manager.h>
#include <violet/object_info.h>
#include <violet/knowledge_base.h>
#include <violet/util.h>
#include <violet/global_parameters.h>

namespace violet {
namespace inputsources {

static double cloudIntersectionRate(tf::Point existingLocation, tf::Point existingSize,
                                    tf::Point newSP, tf::Point newEP,
                                    double range_allowance = 0.005)
{
    tf::Point existingSP; // start points
    tf::Point existingEP; // end points

    existingSP = existingLocation - (existingSize / 2);
    existingEP = existingLocation + (existingSize / 2);

    bool intersectX = util::checkRangesOverlap(existingSP.x(), existingEP.x(), newSP.x(), newEP.x(), range_allowance);
    double rateX = util::rangeIntersectionRate(existingSP.x(), existingEP.x(), newSP.x(), newEP.x());

    bool intersectY = util::checkRangesOverlap(existingSP.y(), existingEP.y(), newSP.y(), newEP.y(), range_allowance);
    double rateY = util::rangeIntersectionRate(existingSP.y(), existingEP.y(), newSP.y(), newEP.y());

    bool intersectZ = util::checkRangesOverlap(existingSP.z(), existingEP.z(), newSP.z(), newEP.z(), range_allowance);
    double rateZ = util::rangeIntersectionRate(existingSP.z(), existingEP.z(), newSP.z(), newEP.z());

    return (intersectX ? rateX : 0.0) *
           (intersectY ? rateY : 0.0) *
           (intersectZ ? rateZ : 0.0);
}

class Segmentation : public InputSource
{
    double min_cloud_intersection;
    util::GaussianPdf _pdf;
#ifdef SEGMENTATION_TRANSFORM_POINT
    tf::TransformListener _tf_listener;
#endif
public:
    Segmentation(const std::string source_name);
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr& msg);
};

Segmentation::Segmentation(const std::string source_name): InputSource(source_name)
{
    double confidence_defaults[4] = {0.9, 0.1, 0.45, 0.55};
    std::vector<double> confidence_vec = _param_handle.param<std::vector<double> >("confidence_confusion_matrix",
                                                                                   std::vector<double>(confidence_defaults, confidence_defaults + 4));

    for(int i = 0; i < 4; ++i) {
        _confidence_confusion[i%2][i/2] = confidence_vec[i];
    }

    double sigma_defaults[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    std::vector<double> sigma_vec = _param_handle.param<std::vector<double> >("location_pdf_sigma_matrix",
                                                                              std::vector<double>(sigma_defaults, sigma_defaults + 9));

    for(int i = 0; i < 9; ++i) {
        _pdf.sigma(i%3, i/3) = sigma_vec[i];
    }

    min_cloud_intersection = _param_handle.param<double>("min_cloud_intersection", 0.1);
}

void Segmentation::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{

    tf::Stamped<tf::Point> location;
    tf::Stamped<tf::Point> object_start;
    tf::Stamped<tf::Point> object_end;
    std::string sensor_frame_id;

    for (violet_msgs::DetectionInfo::_objects_type::const_iterator obj_it = msg->objects.begin(); obj_it != msg->objects.end(); ++obj_it) {
        for (violet_msgs::ObjectInfo::_properties_type::const_iterator prop_it = obj_it->properties.begin(); prop_it != obj_it->properties.end(); ++prop_it) {
            if(prop_it->attribute == "location") {
                location.setX(prop_it->data[0]);
                location.setY(prop_it->data[1]);
                location.setZ(prop_it->data[2]);
                location.frame_id_ = msg->header.frame_id;
                location.stamp_ = msg->header.stamp;
            }
            else if(prop_it->attribute == "min") {
                object_start.setX( prop_it->data[0] );
                object_start.setY( prop_it->data[1] );
                object_start.setZ( prop_it->data[2] );
                object_start.frame_id_ = msg->header.frame_id;
                object_start.stamp_ = msg->header.stamp;
            }
            else if(prop_it->attribute == "max") {
                object_end.setX( prop_it->data[0] );
                object_end.setY( prop_it->data[1] );
                object_end.setZ( prop_it->data[2] );
                object_end.frame_id_ = msg->header.frame_id;
                object_end.stamp_ = msg->header.stamp;
            }
            else if(prop_it->attribute == "sensor_frame") {
                sensor_frame_id = prop_it->values[0];
            }
        }

#ifdef SEGMENTATION_TRANSFORM_POINT
        tf::Stamped<tf::Point> object_start_map;
        tf::Stamped<tf::Point> object_end_map;
        tf::Stamped<tf::Point> location_map;

        try
        {
            _tf_listener.waitForTransform(FIXED_FRAME, REFERENCE_FRAME, ros::Time(0), ros::Duration(1));
            _tf_listener.transformPoint(FIXED_FRAME, object_start, object_start_map);
            _tf_listener.transformPoint(FIXED_FRAME, object_end, object_end_map);
            _tf_listener.transformPoint(FIXED_FRAME, location, location_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN( "Transform Exception: Camera to Map, in Segmentation::callback(). Cause is: %s", ex.what() );
            continue;
        }

        object_start = object_start_map;
        object_end = object_end_map;
        location = location_map;
#endif

        tf::Vector3 size = object_end - object_start;

        KnowledgeBase *kb = KnowledgeBase::instance();
        kb->setLastDetectionTime(_src_name, msg->header.stamp);
        bool matched_existing_object = false;
        const KnowledgeBase::ObjectMap& objs =  kb->objects();
        std::vector<ObjectInfo*> updated_objects;
        for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
            ObjectInfo* cur = it->second;
            if(cloudIntersectionRate(cur->location(), cur->size(), object_start, object_end) >= min_cloud_intersection) {
                cur->setLastDetection(_src_name, msg->header.stamp);
                cur->increaseConfidence(_confidence_confusion);

                util::GaussianPdf update_pdf = cur->locationPDF();
                util::updateGaussianPDF(update_pdf, cur->locationPDF());
                cur->setLocationPDF(update_pdf);
                cur->setLocation(tf::Point(update_pdf.mu(0),  update_pdf.mu(1), update_pdf.mu(2)));
                matched_existing_object = true;
                updated_objects.push_back(cur);
            }
        }

        if(updated_objects.size() == 1) {
            updated_objects[0]->setSize(size);
        }

        if(!global_parameters::add_remove_paused && !matched_existing_object) {
            ObjectInfo *new_object = new ObjectInfo;
            kb->insertObject(new_object);
            new_object->setLocationPDF(util::GaussianPdf(Eigen::Vector3d(location.x(), location.y(), location.z()), _pdf.sigma));
            new_object->setOrientation(tf::Quaternion(0, 0, 0, 1));
            new_object->setSize(size);
            new_object->setLastDetection(_src_name, msg->header.stamp);
            new_object->increaseConfidence(_confidence_confusion);
            new_object->setFovFrame(sensor_frame_id);

            util::GaussianPdf update_pdf = new_object->locationPDF();
            util::updateGaussianPDF(update_pdf, new_object->locationPDF());
            new_object->setLocationPDF(update_pdf);
            new_object->setLocation(tf::Point(update_pdf.mu(0),  update_pdf.mu(1), update_pdf.mu(2)));
        }

    }
}

DEFINE_INPUT_SOURCE(Segmentation, Segmentation)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
