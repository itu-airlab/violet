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
  * @file bayesian_fusion.cpp
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @author Arda İnceoğlu <93arda@gmail.com>
  * @brief Bayesian fusion support classes and functions implementations
  */

#include <violet/bayesian_fusion.h>
/* ROS Libs */
#include <ros/ros.h>
/* Boost */
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

namespace violet {
namespace bayesian_fusion {
ConfusionMatrix constructEmptyConfusionMatrix(const ObjectDatabaseStorageType &objects, double fill_value)
{
    ROS_ASSERT_MSG(objects.size() > 0, "Can't initialize confusion matrix with an empty list");
    ConfusionMatrix mat;
    ConfusionMatrixRow row;
    for(ObjectDatabaseStorageType::const_iterator db_it = objects.begin(); db_it != objects.end(); ++db_it) {
        row[db_it->first] = fill_value;
    }

    for(ConfusionMatrixRow::iterator row_it = row.begin(); row_it != row.end(); ++row_it) {
        mat[row_it->first] = row;
    }

    return mat;
}


ConfusionMatrix constructConfusionMatrixFromCSV(std::string file_name, const ObjectDatabaseStorageType &objects)
{
    ConfusionMatrix mat = constructEmptyConfusionMatrix(objects);
    typedef boost::tokenizer<boost::escaped_list_separator<char> > TokenizerType;

    ROS_INFO("Reading CSV file %s", file_name.c_str());
    std::ifstream file(file_name.c_str());
    std::string csv_row;

    // Read the first row
    std::getline(file, csv_row);
    if(file.fail() && !file.eof()) {
        ROS_FATAL("Confusion matrix first read is failed!");
        std::exit(EXIT_FAILURE);
    }

    TokenizerType tok(csv_row);
    std::vector<std::string> cols;
    for(TokenizerType::const_iterator tok_cur = tok.begin(), tok_end = tok.end(); tok_cur != tok_end; ++tok_cur) {
        cols.push_back(*tok_cur);
        ROS_ASSERT_MSG((*tok_cur == "") || (objects.find(*tok_cur) != objects.end()),
                       "The object name %s in the CSV file %s is not defined in object catalog!",
                       tok_cur->c_str(), file_name.c_str());
    }

    // Assert CSV objects !=
    ROS_ASSERT_MSG((cols.size() - 1) == objects.size(),
                   "The number of objects in CSV file %s and in object catalog doesn't match",
                   file_name.c_str());

    while(std::getline(file, csv_row)) {
        tok.assign(csv_row);

        TokenizerType::const_iterator tok_cur = tok.begin(), tok_end = tok.end();
        std::string col_name;
        for(int i = 0; tok_cur != tok_end; ++tok_cur, ++i) {
            if(i == 0) { // skip first_column
                col_name = *tok_cur;
                continue;
            }
            mat[col_name][cols[i]] = boost::lexical_cast<double>(*tok_cur);
        }
    }

    if(file.fail() && !file.eof()) {
        ROS_FATAL("Confusion matrix read is failed!");
        std::exit(EXIT_FAILURE);
    }

    return mat;
}

void updatePosteriorValues(PosteriorMap &posteriors,
                           const ConfusionMatrix &confusion_matrix,
                           const std::string &observation, double cut_threshold)
{
    ROS_ASSERT_MSG(posteriors.find(observation) != posteriors.end(),
                   "Observation %s is not in the posteriors list", observation.c_str());

    ROS_ASSERT_MSG(confusion_matrix.find(observation) != confusion_matrix.end(),
                   "Observation %s is not in the confusion matrix", observation.c_str());

    if(posteriors[observation].probability > cut_threshold) {
        return;
    }

    double total = 0.0;
    for(PosteriorMap::iterator it = posteriors.begin(); it != posteriors.end(); ++it) {
        const ConfusionMatrixRow& row = confusion_matrix.find(it->first)->second;
        // P(outcome|observation) = P(observation|outcome) * P(outcome)
        it->second.probability = row.find(observation)->second * it->second.probability;
        total += it->second.probability;
    }

    // Normalization
    for(PosteriorMap::iterator it = posteriors.begin(); it != posteriors.end(); ++it) {
        it->second.probability /= total; // WARN: Division by zero may occur here
    }
}

void initializePosteriorValues(PosteriorMap &posteriors, const ObjectDatabaseStorageType &objects)
{
    ROS_ASSERT_MSG(objects.size() > 0, "Can't initialize posterior values with an empty list");
    double value = 1.0 / objects.size();
    for(ObjectDatabaseStorageType::const_iterator it = objects.begin(); it != objects.end(); ++it) {
        posteriors[it->first].probability = value;
        posteriors[it->first].object = &(it->second);
    }
}

} /* END OF NAMESPACE bayesian_fusion */
} /* END OF NAMESPACE violet */
