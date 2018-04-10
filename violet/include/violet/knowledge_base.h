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
  * @file knowledge_base.h
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @author Arda İnceoğlu <93arda@gmail.com>
  * @brief The class declaration of KnowledgeBase
  */

#ifndef KNOWLEDGEBASE_H_
#define KNOWLEDGEBASE_H_

/* Standard Libs */
#include <map>
#include <vector>
/* Internal headers */
#include "enums.h"
#include "util.h"
#include "predicate.h"
#include "defined_predicates.h"
#include "object_info.h"

namespace violet {

#ifdef CLOSED_WORLD_KB
    enum ClosedWorldStates { SCENE_OBJECT, WAITING, MEMORY_OBJECT };
#endif

/**
 * @brief The main implementation of the KnowledgeBase class.
 *
 * KnowledgeBase is the singleton repository of Violet and its interpretation results of the objects. It holds
 * the information in a graph. The nodes of the graph are objects. The edges of the graph are the predicates.
 */
class KnowledgeBase {

public:
    typedef std::map<std::string, const Predicate*> PredicateMap;
    typedef std::map<int, PredicateMap> GraphRow;
    typedef std::map<int, GraphRow> GraphMatrix;
    typedef std::map<int, ObjectInfo*> ObjectMap;

protected:
    /**
     * @brief Singleton instance
     */
    static KnowledgeBase* _instance;
    /**
     * @brief Adjacency matrix for graph.
     *
     * It is stored as map of maps. To access a predicate between two objects use the notation as
     * _graph[FROM][TO]["PREDICATE_NAME"].
     */
    GraphMatrix _graph;

    /**
     * @brief A map of objects
     */
    ObjectMap _objects;
    /**
     * @brief The last ID value that is given to a new object.
     */
    int _max_object_id;

    /**
     * @brief The last detection times for input source algorithms
     */
    std::map <std::string, ros::Time> last_detections;

#ifdef CLOSED_WORLD_KB
    /**
     * @brief The list of the removed objects for resurrection
     */
    ObjectMap _closed_world_objects;
    boost::unordered_map<int, enum ClosedWorldStates> _closed_world_memory;
#endif

    /**
     * @brief KnowledgeBase Constructor
     *
     * The constructor initializes the object IDs and the singleton instance.
     */
    KnowledgeBase();
public:
    /**
      * @brief KnowledgeBase Destructor
      */
    virtual ~KnowledgeBase();
    /**
     * @brief Returns the singleton instance
     * @return KnowledgeBase* A pointer to current instance of KnowledgeBase
     */
    static KnowledgeBase *instance();
    /**
     * @brief Inserts an object (a node) to the graph.
     * @param obj The ObjectInfo pointer for the object.
     * @return int Generated object ID
     */
    int insertObject(ObjectInfo* obj);
    /**
     * @brief Returns the map which is consist of (id, object) pairs
     * @return ObjectMap Object mappings
     */
    const ObjectMap& objects() const;
    /**
     * @brief Return info of a single object which is identified with id
     * @param id Object ID
     * @return ObjectInfo& A reference to ObjectInfo instance of object identified with ID id.
     */
    ObjectInfo& object(int id);
    /**
     * @brief predicates Serialize all predicates in the graph
     * @return The all predicates in the graph as pair of pairs (predicate, (object1, object2) )
     */
    std::vector< std::pair< const Predicate*, std::pair< int, int > > > predicates() const;
    /**
     * @brief Get the predicates for an object identified with id
     * @param id The ID value
     * @return std::vector< pair<const Predicate*, int> > A vector of pairs of (predicate, object_id)
     */
    std::vector< std::pair< const Predicate*, int > > predicates(int id) const;
    /**
     * @brief Get object pairs in graph holding a particular predicate
     * @param _predicate The predicate to look for
     * @return std::vector< pair<int, int> > Matched object id pairs
     * This method searches a predicate in the graph and returns the pairs of objects
     * that have the predicate as a relation.
     */
    std::vector< std::pair<int, int> > predicates(Predicate &_predicate) const;
    /**
     * @brief Returns a vector of predicates that other objects has on the object which is identified with id
     * @param id The object id whose reverse predicates are being queried
     * @return std::vector< std::pair<const Predicate*, int> > The vector of pairs (predicate, object_id)
     */
    std::vector< std::pair<const Predicate*, int> > reversePredicates(int id) const;

    /**
     * @brief Sets the time when the Knowledge base is updated last by the input source source_name
     * @param source_name The name of the source
     * @param _time The time value
     */
    void setLastDetectionTime(std::string source_name,ros::Time _time);
    /**
     * @brief Returns the time when the Knowledge base is updated last by the input source source_name
     * @param source_name The name of the source. (Like "Hand1")
     * @return ros::Time The detection time.
     */
    ros::Time getLastDetectionTime(std::string source_name);
    /**
     * @brief Deletes an object and all predicates connected to it.
     * @param id object id.
     */
    void deleteObject(int id);

    /**
     * @brief Inserts an unary predicate to an object
     * @param predicate The predicate
     * @param object_id The object
     */
    void insertPredicate(const Predicate& predicate, int object_id);
    /**
     * @brief Inserts a binary predicate from object1 to object2
     * @param predicate The predicate
     * @param object1 First Object
     * @param object2 Second Object
     */
    void insertPredicate(const Predicate& predicate, int object1, int object2);
    /**
     * @brief Deletes a unary predicate from Knowledge Base for an object
     * @param predicate The predicate
     * @param object_id The object
     */
    void deletePredicate(const Predicate& predicate, int object_id);
    /**
     * @brief Deletes a binary predicate from Knowledge Base.
     *
     * The binary predicate directon is from object1 to object2.
     *
     * @param predicate The predicate
     * @param object1 First object
     * @param object2 Second object
     */
    void deletePredicate(const Predicate& predicate, int object1, int object2);

#ifdef CLOSED_WORLD_KB
    /**
     * @brief Checks for the changes in objects and updates the knowledge base if any objects
     * are previously stored as a close world object and the change matches it.
     */
    void updateClosedWorldObjects();
    /**
     * @brief Returns the map of (id, closed_world_object) pairs
     * @return ObjectMap Object mappings
     */
    const ObjectMap& closedWorldObjects() const;
#endif

};

} /* END NAMESPACE violet */

#endif // KNOWLEDGEBASE_H_
