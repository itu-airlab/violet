#include <string>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <violet_msgs/DetectionInfo.h>
#include <violet_srvs/RegisterSource.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "violet_predefined_object_loader");
    ros::NodeHandle nh, loc_nh("~");

    // Violet namespace
    std::string violet_ns = loc_nh.param<std::string>("violet_namespace", "violet");
    // Message frame
    std::string world_frame = loc_nh.param<std::string>("world_frame", "base");
    // Frame published as sensor frame
    std::string simulated_sensor_frame = loc_nh.param<std::string>("simulated_sensor_frame", "camera_rgb_optical_frame");

    ROS_INFO("Waiting for Violet starts up");
    bool service_exists = false;
    for ( int i = 0; i < 3 && !(service_exists = ros::service::exists(violet_ns + "/register_source", false)); ++i)
        { boost::this_thread::sleep_for(boost::chrono::milliseconds(200)); }

    if(service_exists) {
        ros::ServiceClient registrationClient = nh.serviceClient<violet_srvs::RegisterSource>(violet_ns + "/register_source");
        violet_srvs::RegisterSource register_srv;

        register_srv.request.topic_name = violet_ns + "/predefined_objects";
        register_srv.request.source_algorithm_name = "PredefinedObjects";
        bool registered_to_violet = registrationClient.call(register_srv);
        ROS_INFO("Violet registration is %s ", (registered_to_violet ? "successful" : "unsucessful") );
    }

    ros::Publisher pub = nh.advertise<violet_msgs::DetectionInfo>(violet_ns + "/predefined_objects", 1, true);

    violet_msgs::DetectionInfo detections;

    ROS_INFO("Reading predefined objects");
    ROS_ASSERT_MSG(loc_nh.hasParam("predefined_objects"),
                   "The \"predefined_objects\" parameter is not defined. Cannot load objects from parameter server");
    XmlRpc::XmlRpcValue object_list;
    loc_nh.getParam("predefined_objects", object_list);
    ROS_ASSERT_MSG(object_list.getType() == XmlRpc::XmlRpcValue::TypeArray,
                   "\"predefined_objects\" parameter is not an array.");
    // Traverse ~/predefined_objects
    for(int i = 0; i < object_list.size(); ++i) {
        violet_msgs::ObjectInfo obj;
        XmlRpc::XmlRpcValue &current = object_list[i];
        violet_msgs::ObjectProperty name, location, size, sensor_frame;

        name.attribute = "name";
        name.values.push_back(static_cast<std::string>(current["name"]));
        obj.properties.push_back(name);

        location.attribute = "location";
        location.data.push_back(static_cast<double>(current["location"]["x"]));
        location.data.push_back(static_cast<double>(current["location"]["y"]));
        location.data.push_back(static_cast<double>(current["location"]["z"]));
        obj.properties.push_back(location);

        size.attribute = "size";
        size.data.push_back(static_cast<double>(current["size"]["x"]));
        size.data.push_back(static_cast<double>(current["size"]["y"]));
        size.data.push_back(static_cast<double>(current["size"]["z"]));
        obj.properties.push_back(size);

        sensor_frame.attribute = "sensor_frame";
        sensor_frame.values.push_back(simulated_sensor_frame);
        obj.properties.push_back(sensor_frame);

        detections.objects.push_back(obj);
    }

    ROS_INFO("Publishing objects into %s", (violet_ns + "/predefined_objects").c_str());
    detections.header.frame_id = world_frame;
    pub.publish(detections);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    return 0;
}
