//
// Created by Shinichi Kinuwaki on 2022-01-04.
//

#pragma once

template<class T>
class RedoUndoOperator {
public:
    int current_index;
    std::vector< std::vector<T> > objects_history;

    RedoUndoOperator( const std::vector<T>& initial_objects ) {
        Reset(initial_objects);
    }

    void Reset( const std::vector<T>& initial_objects ) {
        current_index = 0;
        objects_history.clear();
        objects_history.push_back(initial_objects);
    }

    void PushBack( const std::vector<T>& objects ) {
        if (objects_history.size() > current_index + 1) {
            // reduce
            objects_history.resize(current_index);
        }
        objects_history.push_back(objects);
        current_index++;
    }

    void Undo( std::vector<T>& objects ) {
        if (current_index > 0 ) {
            current_index--;
            objects = objects_history[current_index];
        }
    }

    bool RedoFeasibility() {
       return (objects_history.size() > current_index + 1) ? true : false;
    }

    void Redo( std::vector<T>& objects ) {
        if (objects_history.size() > current_index + 1) {
            current_index++;
            objects = objects_history[current_index];
        }
    }
};
