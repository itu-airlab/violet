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
  * @file object_catalog.h
  * @brief An extensible catalog for storing cached data of all object types declarations
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  */
#ifndef OBJECTCATALOG_H
#define OBJECTCATALOG_H

#include "object_database.h"

namespace violet {

namespace object_catalog
{
    extern ObjectDatabaseStorageType cataloged_objects;
    static inline void initialize(ObjectDatabaseDriver &driver) { cataloged_objects = driver.loadObjects(); }
    const ObjectDatabaseEntry lookup(const std::string &name);
} /* END OF NAMESPACE object_catalog */

} /* END OF NAMESPACE violet */

#endif // OBJECTCATALOG_H
