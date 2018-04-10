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
 * @file predicate.cpp
 * @author Ongun Kanat <ongun.kanat@gmail.com>
 * @author Arda İnceoğlu <93arda@gmail.com>
 * @brief The implementation of Predicate class
 */
#include <violet/predicate.h>

namespace violet {

Predicate::Predicate(const std::string &predicate_name, const PredicateType_T type) :
    _name(predicate_name), _type(type)
{}

Predicate::~Predicate() {}

const std::string Predicate::name() const
{
    return _name;
}

PredicateType_T Predicate::type() const
{
    return _type;
}

bool Predicate::check(const ObjectInfo &)
{
    if(_type != PREDICATE_UNARY) {
        throw std::string("Invalid call for an unary check for a binary predicate");
    }
    return false;
}

bool Predicate::check(const ObjectInfo &, const ObjectInfo &)
{
    if(_type != PREDICATE_BINARY) {
        throw std::string("Invalid call for an binary check for a unary predicate");
    }
    return false;
}

bool Predicate::operator==(const Predicate &other) const
{
    return (_name == other._name) &&  (_type == other._type);
}

std::vector<const Predicate*> Predicate::dependencies() const
{
    return _depending_predicates;
}

std::vector<const Predicate*> Predicate::conflicts() const
{
    return _conflicting_predicates;
}

std::vector<const Predicate*> Predicate::reverseDependencies() const
{
    return _reverse_depending_predicates;
}

std::vector<const Predicate*> Predicate::reverseConflicts() const
{
    return _reverse_conflicting_predicates;
}

void Predicate::addConflict(const Predicate &conflict)
{
    if(_type == PREDICATE_UNARY && conflict._type == PREDICATE_BINARY) {
        throw std::string("Tried to add binary predicate (" + conflict._name + ") as a conflict to unary predicate (" + _name + ")");
    }

    _conflicting_predicates.push_back(&conflict);
}

void Predicate::removeConflict(const Predicate &conflict)
{
    for(std::vector<const Predicate*>::iterator it = _conflicting_predicates.begin(); it != _conflicting_predicates.end(); ++it) {
        if(*(*it) == conflict) {
            _conflicting_predicates.erase(it);
            return;
        }
    }
}

void Predicate::addDependency(const Predicate &dependency)
{
    if(_type == PREDICATE_UNARY && dependency._type == PREDICATE_BINARY) {
        throw std::string("Tried to add binary predicate (" + dependency._name + ") as a dependency to unary predicate (" + _name + ")");
    }

    _depending_predicates.push_back(&dependency);
}

void Predicate::removeDependency(const Predicate &dependency)
{
    for(std::vector<const Predicate*>::iterator it = _depending_predicates.begin(); it != _depending_predicates.end(); ++it) {
        if(*(*it) == dependency) {
            _depending_predicates.erase(it);
            return;
        }
    }
}

void Predicate::addReverseConflict(const Predicate &conflict)
{
    if(_type == PREDICATE_UNARY) {
        throw std::string("Tried to add reverse conflict to unary predicate (" + _name + ")");
    }

    _reverse_conflicting_predicates.push_back(&conflict);
}

void Predicate::removeReverseConflict(const Predicate &conflict)
{
    for(std::vector<const Predicate*>::iterator it = _reverse_conflicting_predicates.begin(); it != _reverse_conflicting_predicates.end(); ++it) {
        if(*(*it) == conflict) {
            _reverse_conflicting_predicates.erase(it);
            return;
        }
    }
}

void Predicate::addReverseDependency(const Predicate &dependency)
{
    if(_type == PREDICATE_UNARY) {
        throw std::string("Tried to add reverse dependency to unary predicate (" + _name + ")");
    }

    _reverse_depending_predicates.push_back(&dependency);
}

void Predicate::removeReverseDependency(const Predicate &dependency)
{
    for(std::vector<const Predicate*>::iterator it = _reverse_depending_predicates.begin(); it != _reverse_depending_predicates.end(); ++it) {
        if(*(*it) == dependency) {
            _reverse_depending_predicates.erase(it);
            return;
        }
    }
}

} /* END NAMESPACE violet */
