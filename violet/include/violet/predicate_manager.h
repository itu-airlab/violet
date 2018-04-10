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
  * @file predicate_manager.h
  * @brief PredicateManager class declaration
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */
#ifndef PREDICATEMANAGER_H
#define PREDICATEMANAGER_H

#include <vector>
#include <map>

#include "enums.h"
#include "predicate.h"

namespace violet {
/**
 * @brief The manager class for all predicates. If a predicate is defined it should be
 * registered to this manager.
 */
class PredicateManager
{
    /**
     * @brief Map of registered predicates, from predicate type to predicate vector.
     */
    static std::map<PredicateType_T , std::vector<Predicate*> > _predicates;

public:
    PredicateManager() { throw std::string("PredicateManager is not meant to be instantiated"); }
    /**
     * @brief Register a predicate to predicate registry
     * @param predicate The predicate variable
     */
    static void registerPredicate(Predicate& predicate)
    { _predicates[predicate.type()].push_back(&predicate); }
    /**
     * @brief Get a list of predicates
     * @param type The type of the predicate. PREDICATE_UNARY or PREDICATE_BINARY
     * @return A vector of Predicate pointers.
     */
    static std::vector<Predicate *> predicates(PredicateType_T type)
    { return _predicates[type]; }
};

} /* END OF NAMESPACE violet */


#endif // PREDICATEMANAGER_H
