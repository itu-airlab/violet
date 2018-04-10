/*
 *  Copyright (C) 2018 Ongun Kanat <ongun.kanat@gmail.com>
 *  Copyright (C) 2018 Istanbul Technical University
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
  *
  */
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <violet/global_parameters.h>
#include <violet/input_source_manager.h>
#include <violet/knowledge_base.h>

namespace violet {
namespace inputsources {


class PredefinedObjects : public InputSource
{
    util::GaussianPdf _pdf;
    tf::TransformListener tf_listener;
public:
    PredefinedObjects(const std::string source_name);
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &msg);
};


PredefinedObjects::PredefinedObjects(const std::string source_name) : InputSource(source_name)
{
    _confidence_confusion[0][0] = 0.9;
    _confidence_confusion[0][1] = 0.1;
    _confidence_confusion[1][0] = 0.1;
    _confidence_confusion[1][1] = 0.9;

    _pdf.sigma << 0.0001, 0,      0,
                  0,      0.0001, 0,
                  0,      0,      0.0001;
}

void PredefinedObjects::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{
    typedef violet_msgs::DetectionInfo::_objects_type::const_iterator MsgObjIterator;
    typedef violet_msgs::ObjectInfo::_properties_type::const_iterator MsgPropIterator;
    for(MsgObjIterator obj_it = msg->objects.begin(), obj_end = msg->objects.end(); obj_it != obj_end; ++obj_it ) {
        const violet_msgs::ObjectInfo &cur_obj = *obj_it;

        tf::Stamped<tf::Point> location, size;
        std::string name, sensor_frame_id;

        for(MsgPropIterator prop_it = cur_obj.properties.begin(), prop_end = cur_obj.properties.end(); prop_it != prop_end; ++prop_it) {
            if(prop_it->attribute == "name") {
                name = prop_it->values[0];
            }
            else if(prop_it->attribute == "location") {
                location.setX(prop_it->data[0]);
                location.setY(prop_it->data[1]);
                location.setZ(prop_it->data[2]);
                location.frame_id_ = msg->header.frame_id;
                location.stamp_ = msg->header.stamp;
            }
            else if(prop_it->attribute == "size") {
                size.setX(prop_it->data[0]);
                size.setY(prop_it->data[1]);
                size.setZ(prop_it->data[2]);
                size.frame_id_ = msg->header.frame_id;
                size.stamp_ = msg->header.stamp;
            }
            else if(prop_it->attribute == "sensor_frame") {
                sensor_frame_id = prop_it->values[0];
            }
        }

        if(tf_listener.resolve(msg->header.frame_id) != tf_listener.resolve(global_parameters::fixed_frame)) {
            try {
                tf::Stamped<tf::Point> transformed_location;
                tf_listener.waitForTransform(global_parameters::fixed_frame, msg->header.frame_id, ros::Time(0), ros::Duration(0.2));
                tf_listener.transformPoint(global_parameters::fixed_frame,  location, transformed_location);
                location = transformed_location;
            }
            catch(tf::TransformException &exc) {
                ROS_WARN("%s could not transform the location predefined object. Cause: %s", __PRETTY_FUNCTION__, exc.what());
                continue;
            }
        }


        KnowledgeBase *kb = KnowledgeBase::instance();
        kb->setLastDetectionTime(_src_name, msg->header.stamp);
        ObjectInfo *obj;
        const KnowledgeBase::ObjectMap& objs =  kb->objects();
        for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
            ObjectInfo* cur = it->second;
            if(util::volumeIntersectionRate(cur->location(), cur->size(), location, size) >= 0.85 ) {
                obj = cur;
                goto update;
            }
        }

        obj = new ObjectInfo;
        kb->insertObject(obj);
        obj->setLocationPDF(util::GaussianPdf(Eigen::Vector3d(location.x(), location.y(), location.z()), _pdf.sigma));
        obj->setSize(size);
        obj->setOrientation(tf::Quaternion(0, 0, 0, 1));
        obj->setFovFrame(sensor_frame_id);
update:
        obj->setLastDetection(_src_name, msg->header.stamp);
        obj->increaseConfidence(_confidence_confusion);
        util::GaussianPdf update_pdf = obj->locationPDF();
        util::updateGaussianPDF(update_pdf, obj->locationPDF());
        obj->setLocationPDF(update_pdf);
        obj->setLocation(tf::Point(update_pdf.mu(0),  update_pdf.mu(1), update_pdf.mu(2)));
    }
}


DEFINE_INPUT_SOURCE(PredefinedObjects, PredefinedObjects);

}
}
