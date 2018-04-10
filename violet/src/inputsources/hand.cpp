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
#include <violet/global_parameters.h>
#include <violet/util.h>
#include <violet/input_source_manager.h>
#include <violet/knowledge_base.h>
#include <violet_msgs/ObjectInfo.h>
#include <violet_msgs/ObjectProperty.h>

namespace violet {
namespace inputsources {

class Hand : public InputSource
{
    std::string REFERENCE_FRAME;
    std::string TARGET_FRAME;

    void updateKnowledgeBase(int id, ros::Time detection_time, std::string frame);
    tf::TransformListener _tf_listener;
    util::GaussianPdf _pdf;
public:
    Hand(const std::string source_name);

    /**
     * @brief callback Update location of the holding object with Gripper location
     * @param msg
     */
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &msg);
};


Hand::Hand(const std::string source_name) : InputSource(source_name), TARGET_FRAME(global_parameters::fixed_frame)
{
    _pdf.sigma << 0.0001, 0,      0,
                  0,      0.0001, 0,
                  0,      0,      0.0001;
}


void Hand::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{
    for (violet_msgs::DetectionInfo::_objects_type::const_iterator obj_it = msg->objects.begin(); obj_it != msg->objects.end(); ++obj_it)
    {
        for (violet_msgs::ObjectInfo::_properties_type::const_iterator prop_it = obj_it->properties.begin(); prop_it != obj_it->properties.end(); ++prop_it)
        {
            if(prop_it->attribute == "holdingObject")
            {
                int id_ = (int) prop_it->data[0];
                if(id_ == -1)
                    return;
                updateKnowledgeBase(id_, msg->header.stamp, msg->header.frame_id );
            }
        }
    }

}

void Hand::updateKnowledgeBase(int id, ros::Time detection_time, std::string frame)
{

    try{
        KnowledgeBase *kb = KnowledgeBase::instance();
        ObjectInfo& obj = kb->object(id);

        geometry_msgs::PoseStamped pose_gripper_link, pose_map_link;
        pose_gripper_link.pose.position.x = 0;
        pose_gripper_link.pose.position.y = 0;
        pose_gripper_link.pose.position.z = 0.01;
        pose_gripper_link.pose.orientation.x = 0;
        pose_gripper_link.pose.orientation.y = 0;
        pose_gripper_link.pose.orientation.z = 0;
        pose_gripper_link.pose.orientation.w = 1;

        pose_gripper_link.header.stamp = detection_time;
        pose_gripper_link.header.frame_id = frame;


        try
        {
            _tf_listener.waitForTransform( TARGET_FRAME, frame, ros::Time(0), ros::Duration(0.5) ); //detection_time
            _tf_listener.transformPose(TARGET_FRAME, pose_gripper_link, pose_map_link);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR( "Transform Exception: Gripper to Map, in Hand::update_knowledgebase(). Cause is: %s", ex.what() );
            return;
        }

        tf::Point tmp_pt;
        tf::pointMsgToTF(pose_map_link.pose.position, tmp_pt);

        obj.setLastDetection(_src_name, detection_time);
        obj.setProbability(0.9999);
        obj.setLocation(tmp_pt);
        obj.setOrientation(tf::Quaternion(0, 0, 0, 1));
        obj.setLocationPDF(util::GaussianPdf(Eigen::Vector3d(tmp_pt.x(), tmp_pt.y(), tmp_pt.z()), _pdf.sigma));
    }catch(std::string s)
    {
        ROS_INFO("Hand - updateKnowledgeBase() :%s", s.c_str());
    }
}

DEFINE_INPUT_SOURCE(HandLeft, Hand)
DEFINE_INPUT_SOURCE(HandRight, Hand)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
