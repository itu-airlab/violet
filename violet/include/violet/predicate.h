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
  * @file predicate.h
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @author Arda İnceoğlu <93arda@gmail.com>
  * @brief Predicate class declaration
  */
#ifndef PREDICATE_H_
#define PREDICATE_H_

/* Standard Libs */
#include <string>
#include <vector>
/* Boost */
#include <boost/function.hpp>
/* Internal Headers */
#include "enums.h"
#include "object_info.h"

namespace violet {
/**
 * @brief The base class for all predicates.
 */
class Predicate {
    const std::string _name;
    const PredicateType_T _type;
    std::vector<const Predicate*> _conflicting_predicates;
    std::vector<const Predicate*> _depending_predicates;
    std::vector<const Predicate*> _reverse_conflicting_predicates;
    std::vector<const Predicate*> _reverse_depending_predicates;
public:
    /**
     * @brief The main constructor
     * @param predicate_name The name of the predicate
     * @param type Predicate type
     */
    Predicate(const std::string &predicate_name, const PredicateType_T type);
    virtual ~Predicate();
    /**
     * @brief Get name of the predicate
     * @return Name of the predicate (e.g. "on")
     */
    const std::string name() const;
    /**
     * @brief Get type of the predicate
     * @return Type of the predicate (e.g. PRD_BINARY)
     */
    PredicateType_T type() const;
    /**
     * @brief Main routine for investigation of existance of a unary predicate
     * @param object The object
     * @return True if Object conditions are met for the predicate
     */
    virtual bool check(const ObjectInfo &object);
    /**
     * @brief Main routine for investigation of existance of a binary predicate
     * @param object1 First object
     * @param object2 Second object
     * @return True if Object conditions are met for the predicate
     */
    virtual bool check(const ObjectInfo &object1, const ObjectInfo &object2);
    /**
     * @brief Check equality of two predicates
     * @param other The other predicate
     * @return Returns true if predicates' names and types are same
     */
    bool operator==(const Predicate &other) const;
    /**
     * @brief Returns the dependencies of the predicate
     * @return A vector of predicate pointers
     */
    std::vector<const Predicate*> dependencies() const;
    /**
     * @brief Returns the conflicts of the predicate
     * @return A vector of predicate pointers
     */
    std::vector<const Predicate*> conflicts() const;
    /**
     * @brief Returns a list of reverse dependencies of this predicate
     * @return A vector of predicate pointers
     */
    std::vector<const Predicate*> reverseDependencies() const;
    /**
     * @brief Returns a list of reverse conflicfts of this predicate
     * @return A vector of predicate pointers
     */
    std::vector<const Predicate*> reverseConflicts() const;

protected:
    /**
     * @brief Set a predicate as conflicting with this predicate
     * @param conflict The conflicting predicate
     */
    void addConflict(const Predicate& conflict);
    /**
     * @brief Remove a conflicting predicate from this predicate
     * @param conflict The conflicting predicate
     */
    void removeConflict(const Predicate& conflict);
    /**
     * @brief Set a predicate as a dependency of this predicate
     * @param dependency The dependency predicate
     */
    void addDependency(const Predicate& dependency);
    /**
     * @brief Removes a dependency from this predicate
     * @param dependency The dependency predicate
     */
    void removeDependency(const Predicate& dependency);
    /**
     * @brief Set a predicate which is owned by second object of this predicate as conflicting with this binary predicate.
     *
     * For example "top_clear" is the reverse conflict of "on". If object1 is "on" object2 then object2's "top_clear" predicate
     * should be removed.
     *
     * @param conflict The conflicting predicate
     */
    void addReverseConflict(const Predicate& conflict);
    /**
     * @brief Remove a reverse conflict from this predicate
     * @param conflict The conflicting predicate
     */
    void removeReverseConflict(const Predicate& conflict);
    /**
     * @brief Set a predicate which is owned by second object of this predicate as a dependency to this binary predicate.
     *
     * For example "floating" is a reverse dependency of the "not_under".
     * @param dependency Dependency predicate
     */
    void addReverseDependency(const Predicate& dependency);
    /**
     * @brief removeReverseDependency Removes a reverse dependency from the predicate
     * @param dependency Dependency predicate
     */
    void removeReverseDependency(const Predicate& dependency);
};

} /* END NAMESPACE violet */

#endif // PREDICATE_H_
