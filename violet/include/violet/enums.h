/*
 *  Copyright (C) 2017, 2018 Ongun Kanat <ongun.kanat@gmail.com>
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
  * @file common.h
  * @brief Common datatypes for Violet
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */

#include <string>

#ifndef COMMON_H
#define COMMON_H

/**
 * @brief The global namespace for Violet
 */
namespace violet {

/**
 * @brief Enumeration for template attributes
 */
enum TemplateAttribute_T { T_ATTR_NAME, T_ATTR_TYPE, T_ATTR_COLOR, T_ATTR_SIZE, T_ATTR_SHAPE, T_ATTR_MATERIAL};
/**
 * @brief Enumeration for object attributes
 */
enum ObjectAttribute_T { O_ATTR_NAME, O_ATTR_TYPE, O_ATTR_COLOR, O_ATTR_SHAPE, O_ATTR_MATERIAL};

/**
 * @brief Predicate types
 */
enum PredicateType_T {PREDICATE_UNARY, PREDICATE_BINARY};

} /* END OF NAMESPACE violet */

#endif // COMMON_H

