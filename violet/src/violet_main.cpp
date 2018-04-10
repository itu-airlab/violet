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
 * @file violet_main.cpp
 * @author Arda İnceoğlu <93arda@gmail.com>
 * @author Ongun Kanat <ongun.kanat@gmail.com>
 * @brief The implementation of the main event loops
 */
/* Standard Libs */
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <vector>
/* ROS Libs */
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
/* Messages and Services */
#include <std_srvs/Empty.h>
#include <violet_msgs/DetectionInfo.h>
#include <violet_srvs/RegisterSource.h>
#include <violet_msgs/WorldState.h>
#include <violet_msgs/Predicate.h>
#include <violet_msgs/Object.h>
/* Package's Headers */
#include <violet/global_parameters.h>
#include <violet/enums.h>
#include <violet/util.h>
#include <violet/input_source_manager.h>
#include <violet/knowledge_base.h>
#include <violet/predicate_manager.h>
#include <violet/defined_predicates.h>
#include <violet/object_catalog.h>
#include <violet/color.h>

namespace violet {

/**
 * @brief Implementation of main loop
 */
class VioletMain
{
protected:
    /**
     * @brief Global node handle
     */
    ros::NodeHandle node_handle;
    /**
     * @brief Local node handle (/violet by default)
     */
    ros::NodeHandle local_node_handle;

    /**
     * @brief Namespace
     */
    std::string violet_ns;

    /**
     * @brief subscribe_source_srv Source registration service
     */
    ros::ServiceServer subscribe_source_srv;
    /**
     * @brief sourceRegistrationCallback The ROS callback for  source registration service calls
     * @param req Request
     * @param res Response
     * @return According to ROS conventions true for success, false for otherwise
     */
    bool sourceRegistrationCallback(violet_srvs::RegisterSource::Request &req, violet_srvs::RegisterSource::Response &res );
    /**
     * @brief subscribers List of ROS Subscribers created via input source registration
     */
    std::vector<ros::Subscriber*> subscribers;

    /**
     * @brief ROS service server for pausing addition or removal of the objects in the scene.
     */
    ros::ServiceServer add_remove_pause_srv;
    bool addRemovePauseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * @brief world_state_pub The publisher for violet_msgs/WorldState
     */
    ros::Publisher world_state_pub;

    /**
     * @brief kb Pointer to current KnowledgeBase instance.
     *
     * It holds a pointer to current KnowledgeBase. If the package compiled with -DWITH_CLOSED_WORLD_KB option
     * it is initialized to use Closed World KB.
     */
    KnowledgeBase *kb;

    /**
     * @brief Confusion matrix used for decreasing the probabilities of the objects
     */
    boost::array<boost::array<double, 2>,2> confidence_confusion;
public:
    /**
     * @brief SceneInterpreter The constructor for SceneInterpreter class
     *
     * It initializes the subscribers, predicates, node handles and finally the KnowledgeBase
     */
    VioletMain();
    ~VioletMain();

    /**
     * @brief objectRoutineCleaning Drops probability of the objects and removes when probability drops enough
     */
    void objectRoutineCleaning();

    /**
     * @brief publishWorldState Publishes World State Message
     */
    void publishWorldState();

    /**
     * @brief updateKnowledgeBaseGraph
     */
    void updateKnowledgeBaseGraph();

#ifdef CLOSED_WORLD_KB
    void updateClosedWorldKnowledgeBase();
#endif

    /**
     * @brief print_world_state
     */
    void printWorldState();

};



VioletMain::VioletMain() : local_node_handle("~")
{

    global_parameters::fixed_frame = local_node_handle.param<std::string>("fixed_frame", "/map");


    confidence_confusion[0][1] = 0.30;
    confidence_confusion[1][1] = 0.70;

    ROSParamObjectDBDriver object_db_driver;
    object_catalog::initialize(object_db_driver);
    initPredicates();

#ifdef CLOSED_WORLD_KB
    ROS_INFO("Using Closed World KnowledgeBase");
#endif
    kb = KnowledgeBase::instance();
    subscribe_source_srv = local_node_handle.advertiseService("register_source", &VioletMain::sourceRegistrationCallback, this);
    add_remove_pause_srv = local_node_handle.advertiseService("pause_add_remove", &VioletMain::addRemovePauseCallback, this);

    violet_ns = local_node_handle.param<std::string>("namespace", "violet");
    world_state_pub = node_handle.advertise<violet_msgs::WorldState>(violet_ns + "/world_state", 1);
}

VioletMain::~VioletMain()
{
    for(size_t i = 0; i < subscribers.size(); i++) {
        subscribers[i]->shutdown();
        delete subscribers[i];
    }
}

bool VioletMain::sourceRegistrationCallback(violet_srvs::RegisterSource::Request &req, violet_srvs::RegisterSource::Response &res)
{
    if(req.topic_name != "") {
        try
        {
            InputSource *src = InputSourceManager::constructInstance(req.source_algorithm_name);
            ros::Subscriber* sub = new ros::Subscriber( node_handle.subscribe(req.topic_name, 1, &InputSource::callback, src) );
            subscribers.push_back(sub);
            kb->setLastDetectionTime(src->name(), ros::Time::now());
            res.success = true;
            res.message = "";
            ROS_INFO( "Topic %s has been registered for subscription", req.topic_name.c_str() );
        }
        catch(ros::InvalidNameException)
        {
            res.success = false;
            res.message = "Invalid topic name";
            ROS_ERROR( "Invalid topic name (%s) has been sent.", req.topic_name.c_str() );
        }
        catch(std::string exc)
        {}
    }
    else {
        res.success = false;
        res.message = "Empty topic name";
    }

    return true;
}

bool VioletMain::addRemovePauseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    global_parameters::add_remove_paused = !global_parameters::add_remove_paused; //toggle pause
    return true;
}

void VioletMain::objectRoutineCleaning()
{
    const KnowledgeBase::ObjectMap objs =  kb->objects();
    for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
        ObjectInfo* cur = it->second;
        std::vector< std::pair<const Predicate*, int> > preds = kb->reversePredicates(it->first);

        bool support_another_object = false;
        bool in_fov = false;
        bool is_recognized_by_any = false;

        /*TODO: Generalize with removal blockers in Predicate classes */
        for(std::vector< std::pair<const Predicate*, int> >::const_iterator it_p = preds.begin(); it_p != preds.end(); ++it_p ) {
            const Predicate *p = it_p->first;
            if(*p == predicates::on) {
                support_another_object = true;
                break;
            }

            if(*p == predicates::fov) {
                in_fov = true;
            }
        }

        std::vector<InputSource*>  srcs =  InputSourceManager::sources();
        for(std::vector<InputSource*>::iterator src_it = srcs.begin(); src_it != srcs.end(); ++src_it) {
            ros::Time lastdetection = cur->lastDetectionTime((*src_it)->name());
            is_recognized_by_any = lastdetection != ros::Time(0) && (ros::Time::now() - lastdetection).toSec() < 5;
            if(is_recognized_by_any) {
                break;
            }
        }
        if( !support_another_object && in_fov && !is_recognized_by_any) {
            cur->decreaseConfidence(confidence_confusion);
        }

        if(cur->probability() < 0.0001) {
            kb->deleteObject(it->first);
        }

    }
}

void VioletMain::publishWorldState()
{
    const KnowledgeBase::ObjectMap& objs =  kb->objects();
    const std::vector< std::pair<const Predicate*, std::pair<int, int> > >  preds = kb->predicates();

    violet_msgs::WorldState ws_msg;
    ws_msg.header.stamp = ros::Time::now();

    for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
        ObjectInfo* cur = it->second;
        violet_msgs::Object obj_msg;

        obj_msg.id                = it->first;
        obj_msg.probability       = cur->probability();

        obj_msg.pose.position.x   = cur->location().x();
        obj_msg.pose.position.y   = cur->location().y();
        obj_msg.pose.position.z   = cur->location().z();

        obj_msg.pose.orientation.x = cur->orientation().x();
        obj_msg.pose.orientation.y = cur->orientation().y();
        obj_msg.pose.orientation.z = cur->orientation().z();
        obj_msg.pose.orientation.w = cur->orientation().w();

        obj_msg.size.x            = cur->size().x();
        obj_msg.size.y            = cur->size().y();
        obj_msg.size.z            = cur->size().z();

        obj_msg.type              = cur->attribute(O_ATTR_NAME).first;
        obj_msg.type              = cur->attribute(O_ATTR_TYPE).first;
        obj_msg.color             = cur->attribute(O_ATTR_COLOR).first;
        obj_msg.material          = cur->attribute(O_ATTR_MATERIAL).first;

        ws_msg.objects.push_back(obj_msg);
    }

    for(std::vector< std::pair<const Predicate*, std::pair<int, int> > >::const_iterator it = preds.begin(); it != preds.end(); ++it ) {
        const Predicate *p = it->first;
        std::pair<int, int> objs = it->second;

        violet_msgs::Predicate pred_msg;

        pred_msg.name = p->name();

        if(p->type() == PREDICATE_UNARY) {
            pred_msg.type = violet_msgs::Predicate::UNARY;
            pred_msg.object_id.push_back(objs.first);
        }
        else if(p->type() == PREDICATE_BINARY) {
            pred_msg.type = violet_msgs::Predicate::BINARY;
            pred_msg.object_id.push_back(objs.first);
            pred_msg.object_id.push_back(objs.second);
        }

        ws_msg.predicates.push_back(pred_msg);
    }

    world_state_pub.publish(ws_msg);
}

void VioletMain::updateKnowledgeBaseGraph()
{
    /*
     * For the ones who dare to edit this method:
     * This method contains some absurd conversions between pointers and iterators.
     * It may cause you to spend next month, dreaming nightmares about iterators.
     * It may also prevent you to generate your next generation update algorithm.
     * You are warned. BEWARE!
     */
    const KnowledgeBase::ObjectMap& objs = kb->objects();
    const std::vector<Predicate*> unaries = PredicateManager::predicates(PREDICATE_UNARY);
    const std::vector<Predicate*> binaries = PredicateManager::predicates(PREDICATE_BINARY);

    for(KnowledgeBase::ObjectMap::const_iterator i = objs.begin(); i != objs.end(); ++i) {
        for(KnowledgeBase::ObjectMap::const_iterator j = objs.begin(); j != objs.end(); ++j) {
            if( i->first == j->first ) { // Unary check since the both pointers show same obj Ids
                for(std::vector<Predicate*>::const_iterator k = unaries.begin(); k != unaries.end(); ++k) {
                    // k is Predicate**
                    // i is pair<int, ObjectInfo*>
                    if((*k)->check( *(i->second) ) ) {
                        kb->insertPredicate(*(*k), i->first);
                    }
                    else {
                        try
                        {
                            kb->deletePredicate(*(*k), i->first);
                        }
                        catch(std::string exc)
                        {
                            continue;
                        }
                    }
                }
            }
            else {
                for(std::vector<Predicate*>::const_iterator k = binaries.begin(); k != binaries.end(); ++k) {
                    // k is Predicate **
                    // i is pair<int, ObjectInfo*> as well as j
                    if((*k)->check(*(i->second), *(j->second) ) ) {
                        kb->insertPredicate(*(*k), i->first, j->first);
                    }
                    else {
                        try
                        {
                            kb->deletePredicate(*(*k), i->first, j->first);
                        }
                        catch(std::string exc)
                        {
                            continue;
                        }
                    }

                }
            }
        }
    }
}

#ifdef CLOSED_WORLD_KB
void VioletMain::updateClosedWorldKnowledgeBase()
{
    kb->updateClosedWorldObjects();
}
#endif

void VioletMain::printWorldState()
{
    const KnowledgeBase::ObjectMap& objs =  kb->objects();
    const std::vector< std::pair<const Predicate*, std::pair<int, int> > >  preds = kb->predicates();

    std::cout << CLRSCRN;
    if(global_parameters::add_remove_paused) {
        std::cout << RED << BOLD("#### OBJECT ADDITIONS and REMOVALS are PAUSED #####") << RST << std::endl;
    }
#ifdef CLOSED_WORLD_KB
    const KnowledgeBase::ObjectMap& cw_objs =  kb->closedWorldObjects();
    std::cout << BOLD("Closed World Objects:") << RST << std::endl;
    for(KnowledgeBase::ObjectMap::const_iterator it = cw_objs.begin(); it != cw_objs.end(); ++it ) {
        std::cout << it->first << ":(" << RED << it->second->attribute(O_ATTR_TYPE).first  << RST << "," << RST
                                       << GRN << it->second->attribute(O_ATTR_COLOR).first << RST << ") "<< RST;
    }
    std::cout << std::endl << BOLD("--------------------") << RST << std::endl;
    std::cout << BOLD("Scene Objects:") << RST << std::endl;
#endif
    for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
        ObjectInfo* cur = it->second;

        std::cout << BOLD("Object ") <<  it->first << std::endl;

        std::string type  = cur->attribute(O_ATTR_TYPE).first;
        std::string color = cur->attribute(O_ATTR_COLOR).first;

        std::cout << BOLD("Type  :") <<  RED << (type != "" ? type : "(none)") << RST;
        std::cout << BOLD(" Color :") <<  GRN << (color != "" ? color: "(none)") << RST;

        std::cout << BOLD(" Probability :") <<  BLU << std::setw(4) << cur->probability() << RST << std::endl;

        std::cout << BOLD("Location x:") << YEL << std::setw(7) << std::fixed << std::setprecision(5) << cur->location().x() << RST
                  << BOLD(" y:") << YEL << std::setw(7) << std::fixed << std::setprecision(5) << cur->location().y() << RST
                  << BOLD(" z:") << YEL << std::setw(7) << std::fixed << std::setprecision(5) << cur->location().z() << RST << std::endl;

        std::cout << BOLD("Orientation x:") << MAG << cur->orientation().x() << RST
                  << BOLD(" y:") << MAG << cur->orientation().y() << RST
                  << BOLD(" z:") << MAG << cur->orientation().z() << RST
                  << BOLD(" w:") << MAG << cur->orientation().w() << RST << std::endl;

        std::cout << BOLD("Size x:") << CYN << cur->size().x() << RST
                  << BOLD(" y:") << CYN << cur->size().y() << RST
                  << BOLD(" z:") << CYN << cur->size().z() << RST << std::endl;

        std::cout << std::endl;
    }

    std::cout << std::endl;

    for(std::vector< std::pair<const Predicate*, std::pair<int, int> > >::const_iterator it = preds.begin(); it != preds.end(); ++it ) {
        const Predicate *p = it->first;
        std::pair<int, int> objs = it->second;

        if(p->type() == PREDICATE_UNARY) {
            std::cout << BLU << BLD << p->name()  << "(" << objs.first << ") " << RST;
        }
        else if(p->type() == PREDICATE_BINARY) {
            std::cout << GRN << BLD << p->name() << "(" << objs.first << "," << objs.second << ") " << RST;
        }
    }

    std::cout << std::endl;
}

} /* END OF NAMESPACE violet */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "violet");
    ros::NodeHandle n;

    try
    {
        violet::VioletMain main_node;

        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            main_node.updateKnowledgeBaseGraph();
#ifdef CLOSED_WORLD_KB
            main_node.updateClosedWorldKnowledgeBase();
#endif
            if(!violet::global_parameters::add_remove_paused) {
                main_node.objectRoutineCleaning();
            }
            main_node.printWorldState();
            main_node.publishWorldState();
            rate.sleep();
        }
    }
    catch(std::string exc)
    {
        ROS_FATAL("Unhandled exception \"%s\" has been thrown Violet now shut down", exc.c_str());
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    catch (std::exception exc)
    {
        ROS_FATAL("Unhandled exception \"%s has been thrown Violet now shut down", exc.what());
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}
