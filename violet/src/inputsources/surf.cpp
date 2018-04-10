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
  * @file surf.cpp
  * @brief Surf recognizer input source interpreter
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <violet/global_parameters.h>
#include <violet/input_source_manager.h>
#include <violet/object_info.h>
#include <violet/util.h>
#include <violet/object_catalog.h>
#include <violet/bayesian_fusion.h>
#include <violet/knowledge_base.h>
#include <violet_msgs/DetectionInfo.h>

namespace violet {
namespace inputsources {

class Surf : public InputSource
{
    void parseAttributes(const violet_msgs::ObjectInfo &object, ObjectDatabaseEntry &object_attributes, tf::Point &location);
    tf::TransformListener tf_listener;
    bayesian_fusion::ConfusionMatrix confusion_matrix;
    util::GaussianPdf source_pdf;
    double min_volumetric_intersection;

    bayesian_fusion::ConfusionMatrix initializeConfusionMatrix();
public:
    Surf(const std::string source_name);
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &msg);
};

Surf::Surf(const std::string source_name) : InputSource(source_name),
               min_volumetric_intersection(_param_handle.param<double>("min_volumetric_intersection", 0.25)),
               confusion_matrix(initializeConfusionMatrix())
{
    double sigma_defaults[9] = {0.03, 0, 0, 0, 0.02, 0, 0, 0, 0.03};
    std::vector<double> sigma_vec = _param_handle.param<std::vector<double> >("location_pdf_sigma_matrix",
                                                                              std::vector<double>(sigma_defaults, sigma_defaults + 9));

    for(int i = 0; i < 9; ++i) {
        source_pdf.sigma(i%3, i/3) = sigma_vec[i];
    }

    double confidence_defaults[4] = {0.55, 0.45, 0.4, 0.6};
    std::vector<double> confidence_vec = _param_handle.param<std::vector<double> >("confidence_confusion_matrix",
                                                                                   std::vector<double>(confidence_defaults, confidence_defaults + 4));

    for(int i = 0; i < 4; ++i) {
        _confidence_confusion[i%2][i/2] = confidence_vec[i];
    }

}


bayesian_fusion::ConfusionMatrix Surf::initializeConfusionMatrix()
{
    std::string file_name;
    file_name = _param_handle.param<std::string>("detection_confusion_matrix_file", "");
    return bayesian_fusion::constructConfusionMatrixFromCSV(file_name, object_catalog::cataloged_objects);
}

void Surf::parseAttributes(const violet_msgs::ObjectInfo &object, ObjectDatabaseEntry &object_attributes, tf::Point &location)
{
    for(violet_msgs::ObjectInfo::_properties_type::const_iterator prop_it = object.properties.begin(); prop_it != object.properties.end(); ++prop_it ){
        if(prop_it->attribute == "recognition") {
            object_attributes = object_catalog::lookup(prop_it->values[0]);
            location = tf::Point(prop_it->data[0], prop_it->data[1], prop_it->data[2]);
        }
    }
}

void Surf::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{

    for(violet_msgs::DetectionInfo::_objects_type::const_iterator object_it = msg->objects.begin(); object_it != msg->objects.end(); ++object_it) {
        tf::Stamped<tf::Point> camera_location, world_location;
        ObjectDatabaseEntry object_database_info;

        parseAttributes(*object_it, object_database_info, camera_location);

        camera_location.frame_id_ = msg->header.frame_id;
        try
        {
            if(!tf_listener.waitForTransform(global_parameters::fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1))) {
                continue;
            }

            tf_listener.transformPoint(global_parameters::fixed_frame, camera_location, world_location);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Transform Exception in source %s: %s to %s, in %s. Cause is: %s", _src_name.c_str(), msg->header.frame_id.c_str(), global_parameters::fixed_frame.c_str(), __PRETTY_FUNCTION__, ex.what());
            continue;
        }

        // We asssume 2d camera finds the front face of the object
        world_location.setX(world_location.x() + object_database_info.size.x() / 2);
        world_location.setY(world_location.y() + object_database_info.size.y() / 2);

        ObjectInfo *object = NULL;
        KnowledgeBase *kb = KnowledgeBase::instance();
        kb->setLastDetectionTime(_src_name, msg->header.stamp);
        const KnowledgeBase::ObjectMap& kb_objects = kb->objects();
        for(KnowledgeBase::ObjectMap::const_iterator it = kb_objects.begin(); it != kb_objects.end(); ++it ) {
            ObjectInfo* kb_object = it->second;
            if(util::volumeIntersectionRate( kb_object->location(), kb_object->size(), world_location, object_database_info.size) > min_volumetric_intersection ) {
                object = kb_object;
                break;
            }
        }

        if(object == NULL) {
            return;
        }

        // Update Confidence
        object->increaseConfidence(_confidence_confusion);

        // Update Location
        util::GaussianPdf old_object_loc_pdf = object->locationPDF();
        util::GaussianPdf detected_object_loc_pdf(Eigen::Vector3d(world_location.x(), world_location.y(), world_location.z()), source_pdf.sigma);
        util::updateGaussianPDF(old_object_loc_pdf, detected_object_loc_pdf);
        object->setLocationPDF(old_object_loc_pdf);
        object->setLocation(tf::Point(old_object_loc_pdf.mu(0),  old_object_loc_pdf.mu(1), old_object_loc_pdf.mu(2)));

        std::string previous_name = object->attribute(O_ATTR_NAME).first;
        // Update Bayesian Attributes
        object->updateAttributes(confusion_matrix, object_database_info.name);
        // Change size information if the class changes
        std::string new_name = object->attribute(O_ATTR_NAME).first;
        if(previous_name != new_name && new_name != "" ) {
            ObjectDatabaseEntry new_object_data = object_catalog::lookup(new_name);
            object->setSize(new_object_data.size);
        }

        object->setLastDetection( _src_name, msg->header.stamp);

    }
}

DEFINE_INPUT_SOURCE(Surf, Surf)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
