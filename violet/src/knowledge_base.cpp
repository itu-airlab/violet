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
 * @file knowledge_base.cpp
 * @authors Ongun Kanat <ongun.kanat@gmail.com>
 * @authors Arda İnceoğlu <93arda@gmail.com>
 * @brief Knowledge Base Implementation
 */

#include <violet/knowledge_base.h>
#include <violet/defined_predicates.h>

namespace violet {
KnowledgeBase* KnowledgeBase::_instance = NULL;

KnowledgeBase::KnowledgeBase()
{
    if(_instance != NULL) {
        throw std::string("KnowledgeBase instance already created");
    }

    _instance = this;
    _max_object_id = 0;
}

KnowledgeBase::~KnowledgeBase()
{
    _instance = NULL;
    for(ObjectMap::iterator it = _objects.begin(); it != _objects.end(); ++it) {
        delete it->second;
    }
}

KnowledgeBase* KnowledgeBase::instance()
{
    if(_instance == NULL) {
        _instance = new KnowledgeBase();
    }
    return _instance;
}

void KnowledgeBase::insertPredicate(const Predicate& predicate, int object_id)
{
    if(predicate.type() == PREDICATE_BINARY) {
        throw std::string("Invalid insertion of a binary predicate as a unary predicate");
    }

    if(_objects.find(object_id) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: insert unary predicate: Object not found");
    }

    _graph[object_id][object_id][predicate.name()] = &predicate;

    std::vector<const Predicate*> deps = predicate.dependencies();
    std::vector<const Predicate*> conflicts = predicate.conflicts();

    //Get dependencies of current predicate and insert them to graph
    for(std::vector<const Predicate*>::iterator it = deps.begin(); it != deps.end(); ++it) {
        _graph[object_id][object_id][(*it)->name()] = (*it);
    }

    // Get conflicts of current predicate and delete them if they exist
    for(std::vector<const Predicate*>::iterator it = conflicts.begin(); it != conflicts.end(); ++it) {
        if( _graph[object_id][object_id].find((*it)->name()) != _graph[object_id][object_id].end()) {
            _graph[object_id][object_id].erase((*it)->name());
        }
    }
}

void KnowledgeBase::insertPredicate(const Predicate &predicate, int object1, int object2)
{
    if(predicate.type() == PREDICATE_UNARY) {
        throw std::string("Invalid insertion of a unary predicate as a binary predicate");
    }

    if(_objects.find(object1) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: insert binary predicate: Object1 not found");
    }

    if(_objects.find(object2) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: insert binary predicate: Object2 not found");
    }

    _graph[object1][object2][predicate.name()] = &predicate;

    std::vector<const Predicate*> deps = predicate.dependencies();
    std::vector<const Predicate*> conflicts = predicate.conflicts();
    std::vector<const Predicate*> rev_deps = predicate.reverseDependencies();
    std::vector<const Predicate*> rev_conflicts = predicate.reverseConflicts();

    //Get dependencies of current predicate and insert them to graph
    for(std::vector<const Predicate*>::iterator it = deps.begin(); it != deps.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            _graph[object1][object1][(*it)->name()] = (*it);
        }
        else {
            _graph[object1][object2][(*it)->name()] = (*it);
        }
    }

    //Get reverse dependencies of current predicate and insert them to graph
    for(std::vector<const Predicate*>::iterator it = rev_deps.begin(); it != rev_deps.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            _graph[object2][object2][(*it)->name()] = (*it);
        }
        else {
            _graph[object2][object1][(*it)->name()] = (*it);
        }
    }

    //Get conflicts of current predicate and delete them from graph
    for(std::vector<const Predicate*>::iterator it = conflicts.begin(); it != conflicts.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            if( _graph[object1][object1].find((*it)->name()) != _graph[object1][object1].end()) {
                _graph[object1][object1].erase((*it)->name());
            }
        }
        else {
            if( _graph[object1][object2].find((*it)->name()) != _graph[object1][object2].end()) {
                _graph[object1][object2].erase((*it)->name());
            }
        }
    }

    //Get reverse conflicts of current predicate and delete them from graph
    for(std::vector<const Predicate*>::iterator it = rev_conflicts.begin(); it != rev_conflicts.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            if( _graph[object2][object2].find((*it)->name()) != _graph[object2][object2].end()) {
                _graph[object2][object2].erase((*it)->name());
            }
        }
        else {
            if( _graph[object2][object1].find((*it)->name()) != _graph[object2][object1].end()) {
                _graph[object2][object1].erase((*it)->name());
            }
        }
    }
}

void KnowledgeBase::deletePredicate(const Predicate &predicate, int object_id)
{
    if(predicate.type() == PREDICATE_BINARY) {
        throw std::string("Invalid deletion of a binary predicate as a unary predicate");
    }

    if(_objects.find(object_id) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: delete unary predicate: Object not found");
    }

    if(_graph[object_id][object_id].find(predicate.name()) == _graph[object_id][object_id].end()) {
        return;
    }

    _graph[object_id][object_id].erase(predicate.name());

    std::vector<const Predicate*> deps = predicate.dependencies();
    std::vector<const Predicate*> conflicts = predicate.conflicts();

    for(std::vector<const Predicate*>::iterator it = deps.begin(); it != deps.end(); ++it) {
        if( _graph[object_id][object_id].find((*it)->name()) != _graph[object_id][object_id].end()) {
            _graph[object_id][object_id].erase((*it)->name());
        }
    }

    for(std::vector<const Predicate*>::iterator it = conflicts.begin(); it != conflicts.end(); ++it) {
        _graph[object_id][object_id][(*it)->name()] = (*it);
    }

}

void KnowledgeBase::deletePredicate(const Predicate &predicate, int object1, int object2)
{
    if(predicate.type() == PREDICATE_UNARY) {
        throw std::string("Invalid deletion of a unary predicate as a binary predicate");
    }

    if(_objects.find(object1) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: delete binary predicate: Object1 not found");
    }

    if(_objects.find(object2) == _objects.end()) {
        throw std::string("KnowledgeBase.cpp: delete binary predicate: Object2 not found");
    }

    if(_graph[object1][object2].find(predicate.name()) == _graph[object1][object2].end()) {
        return;
    }

    _graph[object1][object2].erase(predicate.name());

    std::vector<const Predicate*> deps = predicate.dependencies();
    std::vector<const Predicate*> conflicts = predicate.conflicts();
    std::vector<const Predicate*> rev_deps = predicate.reverseDependencies();
    std::vector<const Predicate*> rev_conflicts = predicate.reverseConflicts();

    for(std::vector<const Predicate*>::iterator it = deps.begin(); it != deps.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            if( _graph[object1][object1].find((*it)->name()) != _graph[object1][object1].end()) {
                _graph[object1][object1].erase((*it)->name());
            }
        }
        else {
            if( _graph[object1][object2].find((*it)->name()) != _graph[object1][object2].end()) {
                _graph[object1][object2].erase((*it)->name());
            }
        }
    }

    for(std::vector<const Predicate*>::iterator it = rev_deps.begin(); it != rev_deps.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            if( _graph[object2][object2].find((*it)->name()) != _graph[object2][object2].end()) {
                _graph[object2][object2].erase((*it)->name());
            }
        }
        else {
            if( _graph[object2][object1].find((*it)->name()) != _graph[object2][object1].end()) {
                _graph[object2][object1].erase((*it)->name());
            }
        }
    }

    for(std::vector<const Predicate*>::iterator it = conflicts.begin(); it != conflicts.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            _graph[object1][object1][(*it)->name()] = (*it);
        }
        else {
            _graph[object1][object2][(*it)->name()] = (*it);
        }
    }

    //Get reverse conflicts of current predicate and delete them from graph
    for(std::vector<const Predicate*>::iterator it = rev_conflicts.begin(); it != rev_conflicts.end(); ++it) {
        if((*it)->type() == PREDICATE_UNARY) {
            _graph[object2][object2][(*it)->name()] = (*it);
        }
        else {
            _graph[object2][object1][(*it)->name()] = (*it);
        }
    }
}

int KnowledgeBase::insertObject(ObjectInfo *object_info)
{
    int id = ++_max_object_id;
    _objects[id] = object_info;
    for(ObjectMap::iterator it = _objects.begin(); it != _objects.end(); ++it) {
        _graph[it->first][id];
        _graph[id][it->first];
    }
#ifdef CLOSED_WORLD_KB
    _closed_world_memory[id] = WAITING;
#endif
    insertPredicate(predicates::clear, id);
    return id;
}

const KnowledgeBase::ObjectMap &KnowledgeBase::objects() const
{
    return _objects;
}

std::vector< std::pair< const Predicate*, std::pair< int, int > > > KnowledgeBase::predicates() const
{
    std::vector< std::pair<const Predicate*, std::pair<int, int> > >  results;

    for(GraphMatrix::const_iterator i = _graph.begin(); i != _graph.end(); ++i) {
        // i is pair<int, GraphRow>
        const GraphRow& cols = i->second; //Get all columns in the row
        for(GraphRow::const_iterator j = cols.begin(); j != cols.end(); ++j ) {
            // j is pair<int, PredicateMap>
            for(PredicateMap::const_iterator k = j->second.begin(); k != j->second.end(); ++k ) {
                // k is pair<string, Predicate*>
                results.push_back( std::pair<const Predicate*, std::pair<int, int> >(k->second, std::pair<int, int>(i->first, j->first) ) );
            }
        }
    }

    return results;
}

std::vector< std::pair<const Predicate*, int> > KnowledgeBase::predicates(int id) const
{
    if( _objects.find(id) == _objects.end() ) {
        throw std::string("KnowledgeBase.cpp: predicates(id): Object not found");
    }

    std::vector< std::pair<const Predicate*, int> >  results;

    const GraphRow& cols = _graph.find(id)->second; //Get all columns in the row
    for(GraphRow::const_iterator i = cols.begin(); i != cols.end(); ++i ) {
        // map's iterator returns pair<Key, Val>. Here i <- <int, PredicateMap>
        for(PredicateMap::const_iterator j = i->second.begin(); j != i->second.end(); ++j ) {
            results.push_back( std::pair<const Predicate *, int>(j->second, i->first) );
        }
    }

    return results;
}

std::vector< std::pair<int, int> > KnowledgeBase::predicates(Predicate &_predicate) const
{
    std::vector< std::pair<int, int> >  results;

    for(GraphMatrix::const_iterator i = _graph.begin(); i != _graph.end(); ++i) {
        // i is pair<int, GraphRow>
        const GraphRow& cols = i->second; //Get all columns in the row
        for(GraphRow::const_iterator j = cols.begin(); j != cols.end(); ++j ) {
            // j is pair<int, PredicateMap>
            for(PredicateMap::const_iterator k = j->second.begin(); k != j->second.end(); ++k ) {
                // k is pair<string, Predicate*>
                if(*(k->second) == _predicate) {
                    results.push_back( std::pair<int, int>(i->first, j->first) );
                }
            }
        }
    }

    return results;
}

std::vector< std::pair<const Predicate*, int> > KnowledgeBase::reversePredicates(int id) const
{
    if( _objects.find(id) == _objects.end() ) {
        throw std::string("KnowledgeBase.cpp: reverse_predicates(id): Object not found");
    }

    std::vector< std::pair<const Predicate*, int> >  results;

    for(ObjectMap::const_iterator it = _objects.begin(); it != _objects.end(); ++it ) {
        int i = it->first;
        GraphMatrix::const_iterator row = _graph.find(i);
        GraphRow::const_iterator col = row->second.find(id);
        if (col == row->second.end()) {
            continue;
        }
        for(PredicateMap::const_iterator j = col->second.begin(); j != col->second.end(); ++j) {
            results.push_back(std::pair<const Predicate*, int>(j->second, i));
        }
    }

    return results;
}

void KnowledgeBase::setLastDetectionTime(std::string source_name, ros::Time _time)
{
    last_detections[source_name] = _time;
}

ros::Time KnowledgeBase::getLastDetectionTime(std::string source_name)
{
    std::map <std::string, ros::Time>::const_iterator it = last_detections.find(source_name);
    if(it != last_detections.end()) {
        return it->second;
    }
    return ros::Time(0);
}

void KnowledgeBase::deleteObject(int id)
{
    if( _objects.find(id) == _objects.end() ) {
        throw std::string("KnowledgeBase: delete object: Object not found");
    }

    const GraphRow& cols = _graph.find(id)->second; //Get all columns in the row
    for(GraphRow::const_iterator i = cols.begin(); i != cols.end(); ++i ) {
        // map's iterator returns pair<Key, Val>. Here i <- <int, PredicateMap>
        PredicateMap pmap = i->second;
        for(PredicateMap::const_iterator j = pmap.begin(); j != pmap.end(); ++j ) {
            if(id == i->first) {
                try
                {
                    deletePredicate(*(j->second), id);
                }
                catch(std::string exc)
                {}
            }
            else {
                try
                {
                    deletePredicate(*(j->second), id, i->first);
                }
                catch(std::string exc)
                {}
            }
        }
    }

    for(ObjectMap::const_iterator it = _objects.begin(); it != _objects.end(); ++it ) {
        int i = it->first;
        GraphMatrix::const_iterator row = _graph.find(i);
        GraphRow::const_iterator col = row->second.find(id);
        PredicateMap pmap = col->second;
        for(PredicateMap::const_iterator j = pmap.begin(); j != pmap.end(); ++j) {
            if(i != id) {
                try
                {
                    deletePredicate(*(j->second), i, id);
                }
                catch(std::string exc)
                {}
            }
        }
    }

    for (GraphMatrix::iterator it = _graph.begin(); it != _graph.end(); ++it) {
        it->second.erase(id);
    }

    _graph.erase(id);

    ObjectInfo *tmp_ptr = _objects[id];
    _objects.erase(id);

#ifdef CLOSED_WORLD_KB
    if(tmp_ptr->isRecognitionThresholdExceeded()) {
        _closed_world_objects[id] = tmp_ptr;
        _closed_world_memory[id] = MEMORY_OBJECT;
    }
    else {
        _closed_world_memory.erase(id);
#endif
        delete tmp_ptr;
#ifdef CLOSED_WORLD_KB
    }
#endif
}

ObjectInfo& KnowledgeBase::object(int id)
{
    if( _objects.find(id) == _objects.end() ) {
        throw std::string("KnowledgeBase.cpp: object(id): Object not found");
    }

    return *(_objects[id]);
}

#ifdef CLOSED_WORLD_KB
const KnowledgeBase::ObjectMap& KnowledgeBase::closedWorldObjects() const
{
    return _closed_world_objects;
}

void KnowledgeBase::updateClosedWorldObjects()
{
    // Scene Object, Memory Object pairs
    typedef std::vector<std::pair<std::pair<int,ObjectInfo*>, std::pair<int,ObjectInfo*> > > MatchList;
    MatchList matched_objects;
    for(ObjectMap::iterator scene_it = _objects.begin(); scene_it != _objects.end(); ++scene_it) {
        if(_closed_world_memory[scene_it->first] == SCENE_OBJECT || !scene_it->second->isRecognitionThresholdExceeded()) {
            continue;
        }
        ObjectInfo *scene_object = scene_it->second;
        ObjectInfo *memorized_object = NULL;
        for(ObjectMap::iterator memory_it = _closed_world_objects.begin(); memory_it != _closed_world_objects.end(); ++memory_it) {
            if(memory_it->second->attribute(O_ATTR_NAME).first == scene_object->attribute(O_ATTR_NAME).first) {
                memorized_object = memory_it->second;
                matched_objects.push_back(std::pair<std::pair<int,ObjectInfo*>, std::pair<int,ObjectInfo*> >(
                                              std::pair<int, ObjectInfo*>(scene_it->first, scene_it->second),
                                              std::pair<int, ObjectInfo*>(memory_it->first, memory_it->second)));
                _closed_world_objects.erase(memory_it);
                break;

            }
        }
        if(memorized_object == NULL) {
            _closed_world_memory[scene_it->first] = SCENE_OBJECT;
        }
    }

    for(MatchList::iterator match_it = matched_objects.begin(); match_it != matched_objects.end(); ++match_it) {
        ObjectInfo *scene_object = match_it->first.second;
        ObjectInfo *memorized_object = match_it->second.second;

        int old_id = match_it->first.first;
        scene_object->mergeAttributes(*memorized_object);

        int new_id = match_it->second.first;
        _objects[new_id] = scene_object;
        for(ObjectMap::iterator kb_it = _objects.begin(); kb_it != _objects.end(); ++kb_it) {
            _graph[kb_it->first][new_id];
            _graph[new_id][kb_it->first];
        }

        const GraphRow& cols = _graph.find(old_id)->second;         //Get all columns in the row
        for(GraphRow::const_iterator graph_row_i = cols.begin(); graph_row_i != cols.end(); ++graph_row_i ) {
            // map's iterator returns pair<Key, Val>. Here i <- <int, PredicateMap>
            PredicateMap pmap = graph_row_i->second;
            for(PredicateMap::const_iterator graph_col_j = pmap.begin(); graph_col_j != pmap.end(); ++graph_col_j ) {
                if(old_id == graph_row_i->first) {
                    try
                    {
                        insertPredicate(*(graph_col_j->second), new_id);
                        deletePredicate(*(graph_col_j->second), old_id);
                    }
                    catch(std::string exc)
                    {}
                }
                else {
                    try
                    {
                        insertPredicate(*(graph_col_j->second), new_id, graph_row_i->first);
                        deletePredicate(*(graph_col_j->second), old_id, graph_row_i->first);
                    }
                    catch(std::string exc)
                    {}
                }
            }
        }

        for(ObjectMap::const_iterator obj_it = _objects.begin(); obj_it != _objects.end(); ++obj_it ) {
            int current_id = obj_it->first;
            GraphMatrix::const_iterator row = _graph.find(current_id);
            GraphRow::const_iterator col = row->second.find(old_id);
            PredicateMap pmap = col->second;
            for(PredicateMap::const_iterator graph_col_j = pmap.begin(); graph_col_j != pmap.end(); ++graph_col_j) {
                if(current_id != old_id) {
                    try
                    {
                        insertPredicate(*(graph_col_j->second), current_id, new_id);
                        deletePredicate(*(graph_col_j->second), current_id, old_id);
                    }
                    catch(std::string exc)
                    {}
                }
            }
        }
        for (GraphMatrix::iterator it = _graph.begin(); it != _graph.end(); ++it) {
            it->second.erase(old_id);
        }
        _graph.erase(old_id);
        _objects.erase(old_id);
        _closed_world_memory[new_id] = SCENE_OBJECT;
        _closed_world_memory.erase(old_id);

        delete memorized_object;
    }
}
#endif

} /* END NAMESPACE violet */
