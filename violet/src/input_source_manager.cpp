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
  * @file input_source_manager.cpp
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @brief The implementation of InputSourceManager class
  */
#include <string>
#include <violet/input_source_manager.h>

namespace violet {
std::map<std::string, boost::function<InputSource*()> > InputSourceManager::_constructors;
std::vector<InputSource*> InputSourceManager::_sources;

InputSourceManager::InputSourceManager()
{
    throw std::string("InputSourceManager is a static only class it should not be instantiated.");
}

void InputSourceManager::registerSource(const char *source_name, boost::function<InputSource*()> source_creator)
{
    _constructors[std::string(source_name)] = source_creator;
}

InputSource* InputSourceManager::constructInstance(const std::string &input_source_name)
{
    if(_constructors.find(input_source_name) == _constructors.end()) {
        throw(std::string("Instance construction for an unknown input source name : " + input_source_name + " is requested." ));
    }

    InputSource *instance = _constructors[input_source_name]();
    _sources.push_back(instance);
    return instance;
}

std::vector<InputSource *> InputSourceManager::sources()
{
    return _sources;
}

DetectionPredicate::DetectionPredicate(std::string input_souce_name) : Predicate("recognized_" + input_souce_name, PREDICATE_UNARY), _input_source_name(input_souce_name)
{
    PredicateManager::registerPredicate(*this);
}

bool DetectionPredicate::check(const ObjectInfo &object)
{
    ros::Duration diff = ros::Time::now() - object.lastDetectionTime(_input_source_name);
    return diff < ros::Duration(2);
}


} /* END OF NAMESPACE violet */
