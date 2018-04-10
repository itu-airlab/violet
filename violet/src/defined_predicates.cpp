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
 * @file defined_predicates.cpp
 * @authors Ongun Kanat <ongun.kanat@gmail.com>
 * @authors Arda İnceoğlu <93arda@gmail.com>
 * @brief The implementation of the defined predicates
 */
#include <violet/global_parameters.h>
#include <violet/defined_predicates.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <violet/predicate_manager.h>

namespace violet {
std::map<PredicateType_T, std::vector<Predicate*> > PredicateManager::_predicates;
namespace predicates {
/* Unary predicates */


OnTablePredicate::OnTablePredicate() : Predicate("on_table", PREDICATE_UNARY)
{
    PredicateManager::registerPredicate(*this);
}

bool OnTablePredicate::check(const ObjectInfo &object)
{
    tf::Point loc = object.location();
    tf::Point sz  = object.size();

    return util::checkRangesOverlap(loc.x() - (sz.x() / 2), loc.x() + (sz.x() / 2), table_start.x(), table_end.x()) &&
           util::checkRangesOverlap(loc.y() - (sz.y() / 2), loc.y() + (sz.y() / 2), table_start.y(), table_end.y()) &&
           (std::abs((loc.z() - (sz.z() / 2)) - table_start.z()) < 0.005);
}

OnTablePredicate on_table;

Predicate holding("holding", PREDICATE_UNARY);
Predicate clear("clear", PREDICATE_UNARY);

FovPredicate::FovPredicate() : Predicate("fov", PREDICATE_UNARY), listener(NULL)
{
    PredicateManager::registerPredicate(*this);
}

bool FovPredicate::check(const ObjectInfo &object)
{
    if(!listener) {
        throw(std::string("FovPredicate::check listener does not initialized"));
    }

    tf::Stamped<tf::Pose> map_pose(tf::Pose(object.orientation(), object.location() ), object.lastDetectionTime("LineMod"), global_parameters::fixed_frame);
    tf::Stamped<tf::Pose> cam_pose;

    try
    {
        listener->waitForTransform(object.fovFrame(), global_parameters::fixed_frame, ros::Time(0), ros::Duration(0.1));
        listener->transformPose(object.fovFrame(), ros::Time(0), map_pose, global_parameters::fixed_frame, cam_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR( "Transform Exception in FovPredicate::check %s", ex.what() );
        return true;
    }

    tf::Point point = cam_pose.getOrigin();
    bool distance_from_cam   = point.z() >= min_distance_from_camera;
    bool in_horizontal_angle =  std::abs(std::atan2(point.x(), point.z())) <= max_horizontal_angle;
    bool in_vertical_angle   = std::abs(std::atan2(point.y(), point.z())) <= max_vertical_angle;
    return distance_from_cam && in_horizontal_angle && in_vertical_angle;
}

void FovPredicate::initListener()
{
    listener = new tf::TransformListener();
}

FovPredicate::~FovPredicate()
{
    if(listener) {
        delete listener;
    }
}
FovPredicate fov;

/* Binary Predicates */

/* On */

OnPredicate::OnPredicate() : Predicate("on", PREDICATE_BINARY)
{
    PredicateManager::registerPredicate(*this);
    addConflict(on_table);
    addReverseConflict(clear);
    addReverseConflict(fov);
}

bool OnPredicate::check(const ObjectInfo &object1, const ObjectInfo &object2)
{
    tf::Point firstLoc = object1.location();
    tf::Point secondLoc = object2.location();

    tf::Point firstSize = object1.size();
    tf::Point secondSize = object2.size();

    double hs = firstSize.z() + secondSize.z() / 2; // half of sum of sizes (effective distance measure)
    double dz = firstLoc.z() - secondLoc.z(); // Difference of heights

    bool intersectX = std::abs(secondLoc.x() - firstLoc.x()) < (secondSize.x() / 2 + 0.005);
    bool intersectY = std::abs(secondLoc.y() - firstLoc.y()) < (secondSize.y() / 2 + 0.005);

    bool high_enough = (1 - lower_limit_multiplier) * hs <= dz;

    bool touching = dz > 0 && // first above second AND
                    dz <= (1 + upper_limit_multiplier) * hs;

    return intersectX && intersectY && high_enough && touching;
}
OnPredicate on;

/* Near */

NearPredicate::NearPredicate() : Predicate("near", PREDICATE_BINARY)
{
    PredicateManager::registerPredicate(*this);
}

bool NearPredicate::check(const ObjectInfo &object1, const ObjectInfo &object2)
{
    tf::Point firstLoc = object1.location();
    tf::Point secondLoc = object2.location();

    tf::Point firstSize = object1.size();
    tf::Point secondSize = object2.size();

    tf::Point secondSP = secondLoc - secondSize / 2;
    tf::Point secondEP = secondLoc + secondSize / 2;
    tf::Point firstSP = firstLoc - firstSize / 2;
    tf::Point firstEP = firstLoc + firstSize / 2;

    return (
               !(util::checkRangesOverlap( secondSP.x(), secondEP.x(), firstSP.x(), firstEP.x() ) &&
                 util::checkRangesOverlap( secondSP.y(), secondEP.y(), firstSP.y(), firstEP.y() ) &&
                 util::checkRangesOverlap( secondSP.z(), secondEP.z(), firstSP.z(), firstEP.z() )
                 ) &&
               (std::abs( firstLoc.x() - secondLoc.x() ) < ( firstSize.x() + secondSize.x() ) ) &&
               (std::abs( firstLoc.y() - secondLoc.y() ) < ( firstSize.y() + secondSize.y() ) ) &&
               (std::abs( firstLoc.z() - secondLoc.z() ) < 0.01 /*(firstSize.z + secondSize.z)*/)
               );
}
NearPredicate near;



} /* END OF NAMESPACE predicates */

void initPredicates()
{
    ros::NodeHandle nh("~");

    /* FOV predicate parameter initialization */
    predicates::fov.initListener();
    predicates::fov.setParameters(
                    nh.param<double>("predicate_options/fov/minimum_distance_from_camera", 0.52),
                    nh.param<double>("predicate_options/fov/maximum_fov_angle_horizontal", 0.50614548307),
                    nh.param<double>("predicate_options/fov/maximum_fov_angle_vertical", 0.39269908169));

    /* On table predicate initialization */
    tf::Point table_start(
                nh.param<double>("predicate_options/on_table/table_start/x", -0.2),
                nh.param<double>("predicate_options/on_table/table_start/y", -0.56),
                nh.param<double>("predicate_options/on_table/table_start/z", -0.07));
    tf::Point table_end(
                nh.param<double>("predicate_options/on_table/table_end/x", 0.2),
                nh.param<double>("predicate_options/on_table/table_end/y", 0.24),
                nh.param<double>("predicate_options/on_table/table_end/z", -0.07));
    predicates::on_table.initTableLocation(table_start, table_end);

    /* On predicate initialization */
    double on_pred_upper_mult = nh.param<double>("predicate_options/on/upper_limit_multiplier", 0.2);
    double on_pred_lower_mult = nh.param<double>("predicate_options/on/lower_limit_multiplier", 0.3);
    predicates::on.setParameters(on_pred_upper_mult, on_pred_lower_mult);
}

} /* END OF NAMESPACE violet */
