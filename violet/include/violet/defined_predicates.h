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
  * @file defined_predicates.h
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @author Arda İnceoğlu <93arda@gmail.com>
  * @brief This file contains the constant predicate declarations for Violet.
  * To add a predicate insert its class declaration and an extern variable.
  */
#ifndef DEFINEDPREDICATES_H
#define DEFINEDPREDICATES_H

#include <tf/transform_listener.h>
#include "predicate.h"
#include "util.h"

namespace violet {
/**
 * @brief A namespace to seperate defined predicates
 */
namespace predicates {

extern Predicate clear;
extern Predicate holding;

class OnTablePredicate : public Predicate
{
    tf::Point table_start;
    tf::Point table_end;
public:
    OnTablePredicate();
    virtual bool check(const ObjectInfo &object);
    void initTableLocation(tf::Point start, tf::Point end) { table_start = start; table_end = end; }
};

extern OnTablePredicate on_table;

/**
 * @brief Camera field of view detection predicate
 */
class FovPredicate : public Predicate
{
    tf::TransformListener *listener;

    /**
     * @brief The distance at which the Point Cloud data begins in the 3D sensor.
     */
    double min_distance_from_camera;

    /**
     * @brief The angle outside which it is supposed that no accurate matching can be done.
     *
     * The total POV angle will be max_horizontal_angle*2 --> max_horizontal_angle from both sides.
     */
    double max_horizontal_angle;
    /**
     * @brief Same as the max_horizontal_angle but in the vertical axis.
     */
    double max_vertical_angle;


public:
    FovPredicate();
    virtual bool check(const ObjectInfo &object);
    void initListener();
    void setParameters(double min_dist_from_camera_, double max_horizontal_angle_, double max_vertical_angle_)
    {
        min_distance_from_camera = min_dist_from_camera_;
        max_horizontal_angle = max_horizontal_angle_;
        max_vertical_angle = max_vertical_angle_;
    }

    virtual ~FovPredicate();
};
extern FovPredicate fov;

/**
 * @brief Checks wheter the target object is on top of the other
 */
class OnPredicate : public Predicate
{
    double upper_limit_multiplier;
    double lower_limit_multiplier;
public:
    OnPredicate();
    virtual bool check(const ObjectInfo &object1, const ObjectInfo &object2);
    void setParameters(double upper_limit_multiplier_, double lower_limit_multiplier_)
    {
        upper_limit_multiplier = upper_limit_multiplier_;
        lower_limit_multiplier = lower_limit_multiplier_;
    }
};
extern OnPredicate on;

/**
 * @brief Checks near relationship of two objects
 */
class NearPredicate : public Predicate
{
public:
    NearPredicate();
    virtual bool check(const ObjectInfo &object1, const ObjectInfo &object2);
};
extern NearPredicate near;


}/* END OF NAMESPACE predicates */

void initPredicates();

}/* END OF NAMESPACE violet */

#endif // PREDICATES_H
