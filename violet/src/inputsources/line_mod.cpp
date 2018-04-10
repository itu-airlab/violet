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
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <violet/input_source_manager.h>
#include <violet/object_info.h>
#include <violet/util.h>
#include <violet/object_catalog.h>
#include <violet/bayesian_fusion.h>
#include <violet/knowledge_base.h>
#include <violet/global_parameters.h>
#include <violet_msgs/DetectionInfo.h>

namespace violet {
namespace inputsources {
/* Class Definitions */

class LineMod : public InputSource
{
protected:
    // Parameters
    double min_volumetric_intersection;
    // Gaussian Update Base Probability Distribution
    util::GaussianPdf _pdf;
    // Confusion matrices for bayesian updates
    bayesian_fusion::ConfusionMatrix confusion_matrix;

    // Simple parser function
    virtual void parseAttributes(const violet_msgs::ObjectInfo& object,
                                 ObjectDatabaseEntry &object_attributes, tf::Point &location);
    tf::TransformListener tf_listener;

    bayesian_fusion::ConfusionMatrix initializeConfusionMatrix();
public:
    LineMod(const std::string source_name);
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr & msg);
    virtual ~LineMod() {}
};

/* Main Line-Mod implementation */

LineMod::LineMod(const std::string source_name) : InputSource(source_name),
    min_volumetric_intersection(_param_handle.param<double>("min_volumetric_intersection", 0.25)),
    confusion_matrix(initializeConfusionMatrix())
{
    double sigma_defaults[9] = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05};
    std::vector<double> sigma_vec = _param_handle.param<std::vector<double> >("location_pdf_sigma_matrix",
                                                                         std::vector<double>(sigma_defaults, sigma_defaults + 9));

    for(int i = 0; i < 9; ++i) {
        _pdf.sigma(i%3, i/3) = sigma_vec[i];
    }

    double confidence_defaults[4] = {0.8, 0.2, 0.4, 0.6};
    std::vector<double> confidence_vec = _param_handle.param<std::vector<double> >("confidence_confusion_matrix",
                                                                              std::vector<double>(confidence_defaults, confidence_defaults + 4));

    for(int i = 0; i < 4; ++i) {
        _confidence_confusion[i%2][i/2] = confidence_vec[i];
    }
}

void LineMod::parseAttributes(const violet_msgs::ObjectInfo& object, ObjectDatabaseEntry &object_attributes, tf::Point &location)
{
    for (violet_msgs::ObjectInfo::_properties_type::const_iterator prop_it = object.properties.begin(); prop_it != object.properties.end(); ++prop_it) {
        if(prop_it->attribute == "name") {
            object_attributes = object_catalog::lookup(prop_it->values[0].substr(6));
            ROS_ASSERT_MSG(object_attributes.name != "", "Empty object database entry is returned for %s", prop_it->values[0].substr(6).c_str());
        }
        else if(prop_it->attribute == "location") {
            location.setX(prop_it->data[0]);
            location.setY(prop_it->data[1]);
            location.setZ(prop_it->data[2]);
        }
    }
}

bayesian_fusion::ConfusionMatrix LineMod::initializeConfusionMatrix()
{
    std::string file_name;
    file_name = _param_handle.param<std::string>("detection_confusion_matrix_file", "");
    ROS_INFO("File name for input source %s is %s", _src_name.c_str(), file_name.c_str());
    return bayesian_fusion::constructConfusionMatrixFromCSV(file_name, object_catalog::cataloged_objects);
}

void LineMod::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{
    for (violet_msgs::DetectionInfo::_objects_type::const_iterator obj_it = msg->objects.begin(); obj_it != msg->objects.end(); ++obj_it) {
        // Initialize variables
        ObjectDatabaseEntry object_database_info;
        tf::Stamped<tf::Point> detected_location, map_location;

        // Parse message content and match the detection with the database entry
        parseAttributes(*obj_it, object_database_info, detected_location);
        detected_location.frame_id_ = msg->header.frame_id;
        try
        {
            if(!tf_listener.waitForTransform(global_parameters::fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(0.5))) {
                continue;
            }

            tf_listener.transformPoint(global_parameters::fixed_frame, detected_location, map_location);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Transform Exception in source %s: %s to %s, in %s. Cause is: %s", _src_name.c_str(), msg->header.frame_id.c_str(), global_parameters::fixed_frame.c_str(), __PRETTY_FUNCTION__, ex.what());
            continue;
        }

        // Is there any object at this location previously
        ObjectInfo *object = NULL;
        KnowledgeBase *kb = KnowledgeBase::instance();
        kb->setLastDetectionTime(_src_name, msg->header.stamp);
        const KnowledgeBase::ObjectMap& kb_objects =  kb->objects();
        for(KnowledgeBase::ObjectMap::const_iterator it = kb_objects.begin(); it != kb_objects.end(); ++it ) {
            ObjectInfo* kb_object = it->second;
            if(util::volumeIntersectionRate( kb_object->location(), kb_object->size(), map_location, object_database_info.size) > min_volumetric_intersection ) {
                object = kb_object;
                goto update;
            }
        }

        if(global_parameters::add_remove_paused) {
            continue;
        }

        object = new ObjectInfo();
        kb->insertObject(object);
        object->setLocationPDF(util::GaussianPdf(Eigen::Vector3d(map_location.x(), map_location.y(), map_location.z()), _pdf.sigma));
        object->setOrientation(tf::Quaternion(0, 0, 0, 1));
        object->setSize(object_database_info.size);
        object->setFovFrame(msg->header.frame_id);
update:
        // Update Confidence
        object->increaseConfidence(_confidence_confusion);

        // Update Location
        util::GaussianPdf old_object_loc_pdf = object->locationPDF();
        util::GaussianPdf detected_object_loc_pdf(Eigen::Vector3d(map_location.x(), map_location.y(), map_location.z()), _pdf.sigma);
        util::updateGaussianPDF(old_object_loc_pdf, detected_object_loc_pdf);
        object->setLocationPDF(old_object_loc_pdf);
        object->setLocation(tf::Point(old_object_loc_pdf.mu(0),  old_object_loc_pdf.mu(1), old_object_loc_pdf.mu(2)));

        std::string previous_name = object->attribute(O_ATTR_NAME).first;
        // Update Bayesian Attributes
        object->updateAttributes(confusion_matrix, object_database_info.name);
        object->setLastDetection( _src_name, msg->header.stamp);
    }
}


DEFINE_INPUT_SOURCE(LineMod, LineMod)
DEFINE_INPUT_SOURCE(LineModHS, LineMod)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
