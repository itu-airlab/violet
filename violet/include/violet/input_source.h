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
  * @file input_source.h
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @brief InputSource class declaration
  */
#ifndef INPUTSOURCE_H
#define INPUTSOURCE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/function.hpp>
#include <violet_msgs/DetectionInfo.h>


namespace violet {
/**
 * @brief The base class for writing input source interpreters.
 *
 * To implement an input source interpreter override InputSource::callback
 * and use macro #DEFINE_INPUT_SOURCE(name, inputsource).
 */
class InputSource
{
protected:
    /**
     * @brief Input Source's name
     */
    std::string _src_name;
    /**
     * @brief Confusion matrix for confidence value
     */
    boost::array<boost::array<double, 2>, 2> _confidence_confusion;
    /**
      * @brief A ROS node handle whose namespace is `<violet_namespace>/input_source_options/<_src_name>`
      * by default.
      */
    ros::NodeHandle _param_handle;
public:
    InputSource(std::string src_name) : _src_name(src_name), _param_handle("~/input_source_options/" + _src_name) {}
    /**
     * @brief The main callback of the InputSource any class derived from InputSource. Any input
     * source should override this method.
     */
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &) =0;
    /**
     * @brief Get the name of the input source.
     * @return The name of the input source.
     */
    std::string name() const { return _src_name;  }

    boost::array<boost::array<double, 2>, 2> confidence_confusion() { return _confidence_confusion; }


    virtual ~InputSource() {}
};


} /* END OF NAMESPACE violet */


#endif // INPUTSOURCE_H
