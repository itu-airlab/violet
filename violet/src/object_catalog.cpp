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
  * @file object_catalog.cpp
  * @brief An extensible catalog for caching data of all possible objects
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */
#include <violet/object_catalog.h>
#include <map>

namespace violet {
namespace object_catalog {
ObjectDatabaseStorageType cataloged_objects;
const ObjectDatabaseEntry lookup(const std::string &name)
{
    ObjectDatabaseStorageType::iterator it = cataloged_objects.find(name);
    if(it == cataloged_objects.end()) {
        return ObjectDatabaseEntry();
    }

    return it->second;
}

} /* END OF NAMESPACE object_catalog */
} /* END OF NAMESPACE violet */

