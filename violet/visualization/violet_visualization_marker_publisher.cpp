/*
 *  Copyright (C) 2017 Ongun Kanat <ongun.kanat@gmail.com>
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

#include <string>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <violet_msgs/WorldState.h>
#include <visualization_msgs/MarkerArray.h>

struct Color
{
    double r;
    double g;
    double b;
    Color() {}
    Color(double r_, double g_, double b_) : r(r_), g(g_), b(b_) {}
};

std::unordered_map<std::string, Color> color_mapping = {
    {"", Color(1.0, 1.0, 1.0)},
    {"red", Color(1.0, 0.0, 0.0)},
    {"green", Color(0.0, 0.2, 0.0)},
    {"blue", Color(0.0, 0.0, 0.9)},
    {"pink", Color(1.0, 0.08, 0.58)},
    {"yellow", Color(1.0, 1.0, 0.0)},
    {"orange", Color(1.0, 0.5, 0.0)},
    {"purple", Color(0.58, 0.0, 0.827)},
    {"black", Color(0.0, 0.0, 0.0)}
};

std::unordered_map<std::string, int> shape_mapping = {
    {"", visualization_msgs::Marker::CUBE},
    {"prism", visualization_msgs::Marker::CUBE},
    {"cylinder", visualization_msgs::Marker::CYLINDER},
    {"spherical", visualization_msgs::Marker::SPHERE }
};


struct Visualizer
{
    struct Object
    {
        int id;
        double probability;
        tf::Point center;
        tf::Point size;
        std::string name;
        Color color;
        int shape;
    };

    static std::string world_frame;

    std::string source;
    std::vector<Object> objects;
    ros::Publisher publisher;
    void callback(const violet_msgs::WorldState::ConstPtr &msg);
    void publishMarkers();
};


std::string Visualizer::world_frame;

void Visualizer::callback(const violet_msgs::WorldState::ConstPtr &msg)
{
    objects.clear();

    Object to_add;
    for(const violet_msgs::Object &current_obj : msg->objects) {

        to_add.id = current_obj.id;
        tf::pointMsgToTF(current_obj.pose.position, to_add.center);
        to_add.name = current_obj.name;
        tf::pointMsgToTF(current_obj.size, to_add.size);
        to_add.color = color_mapping[current_obj.color];
        to_add.shape = shape_mapping[current_obj.shape];
        to_add.probability = current_obj.probability;
        objects.push_back(to_add);
    }
}

void Visualizer::publishMarkers()
{
    visualization_msgs::MarkerArray marr;
    visualization_msgs::Marker deleteall_marker;
    deleteall_marker.action = 3;
    marr.markers.push_back(deleteall_marker);
    for(int i = 0; i < objects.size(); ++i) {
        Object& obj = objects[i];

        visualization_msgs::Marker object_shape;
        visualization_msgs::Marker centeroid_sphere;
        visualization_msgs::Marker name_text;

        /* Objects */
        object_shape.action = visualization_msgs::Marker::ADD;
        object_shape.header.frame_id = world_frame;
        object_shape.type = obj.shape;
        object_shape.ns = "object_shapes";
        object_shape.id = i;

        object_shape.scale.x = obj.size.x();
        object_shape.scale.y = obj.size.y();
        object_shape.scale.z = obj.size.z();

        object_shape.pose.position.x = obj.center.x();
        object_shape.pose.position.y = obj.center.y();
        object_shape.pose.position.z = obj.center.z();

        object_shape.pose.orientation.x = 0.0;
        object_shape.pose.orientation.y = 0.0;
        object_shape.pose.orientation.z = 0.0;
        object_shape.pose.orientation.w = 1.0;

        object_shape.color.r = obj.color.r;
        object_shape.color.g = obj.color.g;
        object_shape.color.b = obj.color.b;
        object_shape.color.a = 0.6;

        marr.markers.push_back(object_shape);

        /* Centeroids */
        centeroid_sphere.action = visualization_msgs::Marker::ADD;
        centeroid_sphere.header.frame_id = world_frame;
        centeroid_sphere.type = visualization_msgs::Marker::SPHERE;
        centeroid_sphere.ns = "centeroid_spheres";
        centeroid_sphere.id = i;

        centeroid_sphere.scale.x = 0.01;
        centeroid_sphere.scale.y = 0.01;
        centeroid_sphere.scale.z = 0.01;

        centeroid_sphere.pose.position.x = obj.center.x();
        centeroid_sphere.pose.position.y = obj.center.y();
        centeroid_sphere.pose.position.z = obj.center.z();

        centeroid_sphere.pose.orientation.x = 0.0;
        centeroid_sphere.pose.orientation.y = 0.0;
        centeroid_sphere.pose.orientation.z = 0.0;
        centeroid_sphere.pose.orientation.w = 1.0;

        // Negative for maximum visibility
        centeroid_sphere.color.r = 1.0 - obj.color.r;
        centeroid_sphere.color.g = 1.0 - obj.color.g;
        centeroid_sphere.color.b = 1.0 - obj.color.b;
        centeroid_sphere.color.a = 0.7;

        marr.markers.push_back(centeroid_sphere);

        /* Text */
        name_text.action = visualization_msgs::Marker::ADD;
        name_text.header.frame_id = world_frame;
        name_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        std::stringstream ss;

        ss << obj.id << "|"
           << std::setw(4) << std::setprecision(2) << std::fixed << obj.probability
           << std::endl << obj.name;

        name_text.text = ss.str();
        name_text.ns = "name_texts";
        name_text.id = i;

        name_text.scale.x = 0.04;
        name_text.scale.y = 0.04;
        name_text.scale.z = 0.04;

        name_text.pose.position.x = obj.center.x();
        name_text.pose.position.y = obj.center.y() - obj.size.y() / 2 - 0.05;
        name_text.pose.position.z = obj.center.z();

        name_text.pose.orientation.x = 0.0;
        name_text.pose.orientation.y = 0.0;
        name_text.pose.orientation.z = 0.0;
        name_text.pose.orientation.w = 1.0;

        name_text.color.r = 1.0;
        name_text.color.g = 1.0;
        name_text.color.b = 1.0;
        name_text.color.a = 1.0;

        marr.markers.push_back(name_text);
    }
    publisher.publish(marr);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "violet_marker_publisher");
    ROS_INFO("KnowledgeBase marker publisher has started");

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    std::string violet_ns = local_nh.param<std::string>("violet_namespace", "violet");

    Visualizer visualizer;
    visualizer.publisher = nh.advertise<visualization_msgs::MarkerArray>(violet_ns+"/visualization_markers", 1);

    ros::Subscriber kb_sub = nh.subscribe(violet_ns + "/world_state", 1, &Visualizer::callback, &visualizer);

    Visualizer::world_frame = nh.param<std::string>(violet_ns + "/fixed_frame", "/map");

    ROS_INFO("World frame is %s", Visualizer::world_frame.c_str());

    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        visualizer.publishMarkers();
        rate.sleep();
    }

    return 0;
}
