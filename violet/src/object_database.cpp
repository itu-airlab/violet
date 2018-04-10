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
  * @file object_database.cpp
  * @brief An implementation of a basic database driver which uses ROS param
  * server for object catalog
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */
#include <violet/object_database.h>
#include <ros/ros.h>

namespace violet {
ObjectDatabaseStorageType ROSParamObjectDBDriver::loadObjects()
{
    ros::NodeHandle nh(ros_namespace);
    ROS_ASSERT_MSG(nh.hasParam("defined_objects"),
                   "The \"defined_objects\" parameter is not defined. Cannot load objects from parameter server");

    ObjectDatabaseStorageType result;

    XmlRpc::XmlRpcValue object_list;
    nh.getParam("defined_objects", object_list);
    ROS_ASSERT_MSG(object_list.getType() == XmlRpc::XmlRpcValue::TypeArray,
                   "\"defined_object\" parameter is not an array.");
    for(int i = 0; i < object_list.size(); ++i) {
        XmlRpc::XmlRpcValue::iterator current = object_list[i].begin();
        ObjectDatabaseEntry info;
        info.name = current->first;
        XmlRpc::XmlRpcValue attrs = current->second;
        info.type     = attrs.hasMember("type") ? static_cast<std::string>(attrs["type"])         : "";
        info.color    = attrs.hasMember("color") ? static_cast<std::string>(attrs["color"])       : "";
        info.material = attrs.hasMember("material") ? static_cast<std::string>(attrs["material"]) : "";
        info.shape    = attrs.hasMember("shape") ? static_cast<std::string>(attrs["shape"])       : "";
        info.size     = tf::Point(static_cast<double>(attrs["size"]["x"]),
                                  static_cast<double>(attrs["size"]["y"]),
                                  static_cast<double>(attrs["size"]["z"]));
        result[info.name] = info;
    }

    return result;
}

} /* END OF NAMESPACE violet */
