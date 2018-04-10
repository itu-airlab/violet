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
  * @file global_parameters.h
  * @brief Global parameter declarations
  * @author Ongun Kanat
  */

#ifndef GLOBAL_PARAMETERS_H
#define GLOBAL_PARAMETERS_H

#include<string>

namespace violet {
namespace global_parameters {

/**
 * @brief The variable which holds where the object's location info transformed to
 */
extern std::string fixed_frame;

/**
  * @brief A boolean variable denotes whether the additions and removals are paused (e.g.
  * at the moment of manipulation)
  */
extern bool add_remove_paused;
} /* END OF NAMESPACE global_parameters */
} /* END OF NAMESPACE violet */


#endif // GLOBAL_PARAMETERS_H
