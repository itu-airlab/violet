# Using Bayesian Update API

Violet provides a simple API for updating object confidence and identity information for input source interpreters. Though it's possible to update object confidence via setter in **`ObjectInfo`** class, using update functions strongly encouraged for confidence updates. You must use Bayesian identity update API for attribute and object identity updates. This page is only an introduction and developers should check Doxygen API reference and existing implementation for further details of implementation.

## Updating Object Confidence

**`ObjectInfo`** class provides `increaseConfidence` and `decreaseConfidence` methods for updating object confidence. Both of the methods take a 2x2 confusion matrix of double precision floating numbers as input.  The rows of confusion matrix are actual values and columns denote observed values. Hence, `increaseConfidence`uses the first column of the confusion matrix (i.e `matrix[x][0]`) while `decreaseConfidence` uses the second column. The 2x2 size is a deliberate choice in order to enable users to use same confusion matrix for both `increaseConfidence` and `decreaseConfidence` functions.

`decreaseConfidence` method is automatically called with confusion matrix below for objects that haven't been updated for 5 seconds by default matrix below:

<table>
<tr>
	<th style="border: 0;"></th><th colspan="3">Observed</th>
</tr>
<tr>
	<th rowspan="2">Actual</th>
	<td>0.7</td><td>0.3</td>
</tr>
<tr>
	<td>0.3</td><td>0.7</td>
</tr>
</table>
 
### Confidence Update Example for an Input Source
This is an example code for increasing object confidence. It is advised to look up API documentation for  **`InputSource`** class and documentation for  [adding your own custom input source](adding_input_sources.md).

```c++
#include <violet/input_source.h>
#include <violet/input_source_manager.h>
#include <violet/bayesian_fusion.h>

namespace violet {
namespace inputsources {

class MyInputSourceInterpreter : public InputSource
{
public:
    MyInputSourceInterpreter();
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &msg);
};

MyInputSourceInterpreter::MyInputSourceInterpreter() : 
	InputSource("MyInputSource"),
       local_nh("~"),
       /* The line below reads parameters from ROS parameter /violet/MyInputSource/confusion_matrix */
       confusion_matrix(initializeConfusionMatrix(local_nh)) 
{
    double confidence_defaults[4] = {0.55, 0.45, 0.4, 0.6};

    for(int i = 0; i < 4; ++i) {
        _confidence_confusion[i%2][i/2] = confidence_defaults[i];
    }

}

void MyInputSourceInterpreter::callback(const violet_msgs::DetectionInfo::ConstPtr &msg) 
{
	/* Parse msg here */
	/* ... */
		
	std::vector<ObjectInfo*> updated_objects;
	
	/* Fill in updated_objects here with ObjectInfo pointers for existing objects
	 * in knowledge base and newly allocated ObjectInfo* for new objects */
	/* ... */
	
	for(int i = 0; i < updated_objects.size(); ++i) { // For all objects which are found in msg
		updated_objects[i]->increaseConfidence(_confidence_confusion); // Increase confidence
	}
	
}

DEFINE_INPUT_SOURCE(MyInputSource, MyInputSourceInterpreter)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
```

## Updating Object Identity

The current version of Violet is able to recall defined objects and 5 different attributes (name, type, size, color, material, shape) of each from a database. The identity information of an object can be updated via utility functions in `bayesian_fusion` namespace.

Violet's default database engine recalls object identity and attribute information from ROS parameter `/violet/defined_objects` to create an object catalog. The default configuration file `definitions.yaml` in `config` directory in package is loaded to fill this parameter.  This information can be matched with detected object in callback funtion and resulting database entry is used to update objects attribute. 

`updateAttributes`method of **`ObjectInfo`** class preforms Bayesian attribute update and similar to confidence confusion matrix, it requires a confusion matrix parameter and an extra current observation parameter. In contrast to confusion matrix in confidence update, confusion matrix used in Bayesian identity update functions is a mapping data structure called `bayesian_fusion::ConfusionMatrix`. It can be assumed that `ConfusionMatrix` is a 2D matrix with `std::string` indices.

The required data structure can be generated by calling `bayesian_fusion::constructConfusionMatrixFromCSV` function which takes two parameters: a CSV file path and the object catalog encoded in `ObjectDatabaseStorageType`. The structure of a CSV file is below:

<table>
<tr>
	<th style="border: 0;"></th>
	<th>rubber_duck</th>
	<th>toy_tomato</th>
	<th>blue_cube</th>
</tr>
<tr>
	<th>rubber_duck</th>
	<td>0.9</td><td>0.01</td><td>0.09</td>
</tr>
<tr>
	<th>toy_tomato</th>
	<td>0.2</td><td>0.7</td><td>0.1</td>
</tr>
<tr>
	<th>blue_cube</th>
	<td>0.1</td><td>0.1</td><td>0.8</td>
</tr>
</table>

The row labels are the actual values and the column label are the observed values. The number of rows must be equal to number of columns also it is required that the number of objects and their names should be excatly same with the object catalog defined in `definitions.yaml`; otherwise, Violet _**will shut itself down**_  by an assertion fail.

### Example Code for Object Identity Updates

This examples builds upon previous example of confidence update, the lines below are addition to constructor and callback methods.


```c++
MyInputSourceInterpreter::MyInputSourceInterpreter() : 
	/* member initialization */
{
    /* ... */
    
    /* Create configuration matrix and store it into id_confusion_matrix member of MyInputSourceInterpreter
     * object_catalog::cataloged_objects is the standard Violet database that is read from ROS parameter server
     *
     * config_path is a made-up variable. It can be taken from ROS parameter server as a parameter.
     */
    id_confusion_matrix = bayesian_fusion::constructConfusionMatrixFromCSV(config_path + "my_confusion_matrix.csv",
    															     object_catalog::cataloged_objects);
}

void MyInputSourceInterpreter::callback(const violet_msgs::DetectionInfo::ConstPtr &msg) 
{
	/* ... */
	
	for(int i = 0; i < updated_objects.size(); ++i) { // For all objects which are found in msg
		/* ... */
		
		/* current_observation is a made up string value, it should be parsed/inferred from msg */
		updated_objects[i]->updateAttributes(id_confusion_matrix, current_observation);
	}
	
}
```
