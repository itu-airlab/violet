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

/**
  * @file object_database.g
  * @brief The declarations for drivers reading from object databases
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */

#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

#include <string>
#include <tf/tf.h>
#include <boost/unordered_map.hpp>

namespace violet {

/**
 * @brief A POD structure for storing object data read from a database
 */
struct ObjectDatabaseEntry
{
    std::string name;
    std::string type;
    tf::Point   size;
    std::string color;
    std::string material;
    std::string shape;
    ObjectDatabaseEntry() {}
    ObjectDatabaseEntry(std::string _name,
                        std::string _type,
                        tf::Point   _size,
                        std::string _material,
                        std::string _color) : name(_name), type(_type), size(_size),
                                              color(_color), material(_material) {}
    ObjectDatabaseEntry& operator= (const ObjectDatabaseEntry& other)
    {
        name     = other.name;
        type     = other.type;
        size     = other.size;
        color    = other.color;
        material = other.material;
        return *this;
    }
};

/**
 * @brief A map type the objects are stored
 */
typedef boost::unordered_map<std::string, ObjectDatabaseEntry>  ObjectDatabaseStorageType;

/**
 * @brief An abstract driver class for reading from object databases
 */
class ObjectDatabaseDriver
{
public:
    /**
     * @brief Load objects from database and store into a hash map
     * @return A unordered map of objects with keys as the names
     */
    virtual ObjectDatabaseStorageType loadObjects() = 0;
    virtual ~ObjectDatabaseDriver() {}
};

/**
 * @brief The very basic implementation of ObjectDatabaseDriver which reads from ROS parameter server
 */
class ROSParamObjectDBDriver : public ObjectDatabaseDriver
{
    const std::string ros_namespace;
public:
    /**
     * @brief Construct a ROSParamObjectDBDriver object
     * @param ros_ns The namespace the parameters will be read from
     */
    ROSParamObjectDBDriver(const std::string &ros_ns = "~") : ros_namespace(ros_ns) { }
    virtual ObjectDatabaseStorageType loadObjects();
};


} /* END OF NAMESPACE violet */

#endif // OBJECTDATABASE_H
