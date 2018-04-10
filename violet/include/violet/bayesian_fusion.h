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
  * @file bayesian_fusion.h
  * @brief Bayesian fusion support classes and functions declaration
  * @author Ongun Kanat <ongun.kanat@gmail.com>
  * @author Arda İnceoğlu <93arda@gmail.com>
  */
#ifndef BAYESIAN_FUSION_H
#define BAYESIAN_FUSION_H

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <boost/unordered_map.hpp>

#include "object_database.h"

namespace violet {
/**
 * @brief The umbrella namespace for Bayesian fusion classes and functions
 */
namespace bayesian_fusion {

struct PosteriorCell {
    double probability;
    const ObjectDatabaseEntry *object;
};

/**
 * @brief A list of posterior values for bayesian fusion of object attributes
 */
typedef boost::unordered_map<std::string, PosteriorCell> PosteriorMap;

/**
 * @brief A row of a ConfusionMatrix
 */
typedef boost::unordered_map<std::string, double> ConfusionMatrixRow;
/**
 * @brief A confusion matrix constructed from std::map for an input algorithm to implement bayesian
 * fusion for object attributes.
 */
typedef boost::unordered_map<std::string, ConfusionMatrixRow> ConfusionMatrix;

/**
 * @brief Returns a confusion matrix filled with numbers
 * @param objects A list of storage values
 * @param fill_value A default value to be filled into cells
 * @return A ConfusionMatrix filled with fill_value
 */
ConfusionMatrix constructEmptyConfusionMatrix(const ObjectDatabaseStorageType &objects, double fill_value = 0.0);

/**
 * @brief Creates a confusion matrix using a CSV file
 * @param Input stream
 * @return a confusion matrix
 */
ConfusionMatrix constructConfusionMatrixFromCSV(std::string file_name, const ObjectDatabaseStorageType &objects);

/**
 * @brief Initializes the posterior values of an attribute as a uniform distribution
 * @param posteriors The posterior values to be initialized
 * @param objects The list of possible values
 */
void initializePosteriorValues(PosteriorMap &posteriors, const ObjectDatabaseStorageType &objects);

/**
 * @brief Updates the posterior value list of an attribute by implementing Bayes formula for an
 * observation and using a predefined confusion matrix (e.g. a confusion matrix for an input source).
 * @param posteriors The posterior values of the attribute
 * @param confusion_matrix The confusion matrix
 * @param observation The observed value
 * @param cut_threshold The threshold value that no further increase is allowed
 */
void updatePosteriorValues(PosteriorMap &posteriors,
                           const ConfusionMatrix &confusion_matrix,
                           const std::string &observation, double cut_threshold = 0.9999);
}

} /*END OF NAMESPACE violet */


#endif // BAYESIAN_FUSION_H
