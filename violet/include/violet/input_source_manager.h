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

/**
  * @file input_source_manager.h
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @brief Input source manager class declaration
  */
#ifndef INPUTSOURCEMANAGER_H
#define INPUTSOURCEMANAGER_H

#include <map>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "input_source.h"
#include "predicate_manager.h"

namespace violet {
/**
 * @brief Manager that holds the registry of InputSource based classes.
 *
 * InputSourceManager holds a registry for InputSource type objects. To define and
 * register an input source interpreted by an InputSource derivative class use
 * #DEFINE_INPUT_SOURCE(name, inputsource) macro.
 */
class InputSourceManager
{
    static std::map<std::string, boost::function<InputSource*()> > _constructors;
    static std::vector<InputSource*> _sources;
public:
    InputSourceManager();
    static void registerSource(const char* source_name, boost::function<InputSource *()> source_creator);
    static InputSource* constructInstance(const std::string& input_source_name);
    static std::vector<InputSource*> sources();
};


template<class InputSourceType>
class InputSourceFactory
{
    const std::string _source_name;
    InputSource* constructInputSource()
    {
        return new InputSourceType(_source_name);
    }
public:
    InputSourceFactory(const char* source_name) : _source_name(source_name)
    {
        InputSourceManager::registerSource(source_name,
                                           boost::function<InputSource* ()>(
                                               boost::bind(&InputSourceFactory<InputSourceType>::constructInputSource, this)));
    }
};

class DetectionPredicate : public Predicate
{
    std::string _input_source_name;
public:
    DetectionPredicate(std::string input_souce_name);
    virtual bool check(const ObjectInfo &object);
};

/**
  * @brief Define an input source called with name <b>name</b> and handled by class <b>inputsource</b>
  *
  * This macro is used for defining input source interpreters. It creates an InputSourceFactory
  * instance for each input source and also a DetectionPredicate which is automatically registered
  * to PredicateManager. Placing it below the cpp file where the input source interpreter is
  * defined is enough most of the time.
  *
  * An example usage is:
  * @code{.cpp}
  * DEFINE_INPUT_SOURCE(LeftHand, Hand);
  * @endcode
  *
  */
#define DEFINE_INPUT_SOURCE(name, inputsource) \
        static InputSourceFactory<inputsource> inputsource## _## name## _factory(#name); \
        static DetectionPredicate inputsource## _## name## _predicate(#name);

} /* END OF NAMESPACE violet */

#endif // INPUTSOURCEMANAGER_H
