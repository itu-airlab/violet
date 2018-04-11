# Violet

Violet (**V**isual **I**nterpreter and M**o**de**l**ler for Objects and Rela**t**ions) is a 3D world modelling and sensory fusion package for ROS. Currently it can be built on ROS Indigo or newer. This document provides a quick start guide for how to build package and modify it.


## API Docs (Doxygen)
You can refer to [api docs](url) or generate a local copy using provided doxygen file.

## Installation

To compile and run Violet checkout the repo under your catkin workspace.

### Dependencies

Violet is a self contained ROS package. Its only dependencies are core ROS packages such as `tf` and `std_msgs`. It also uses Boost and Eigen3 libraries.

### Compilation and Running
The default configuration can be compiled like:
```
$ catkin_make
```
The default launch file is started as:
```
$ roslaunch violet violet.launch
```

#### Compilation options

Violet has a compile time boolean option (`WITH_CLOSED_WORLD_KB`) to enable closed world knowledge base. Its default is `ON`.  If you want to disable it pass the option to catkin as below

```
$ catkin_make -DWITH_CLOSED_WORLD_KB=OFF
```


#### Visualizing World State
To visualize published markers in RViz a ROS node is provided. It can be started like:
```
$ rosrun violet violet_visualization_marker_publisher
```

## Tutorials
* [The Event Loop of Violet](event_loop.md)
* [Adding Custom Input Sources](adding_input_sources.md)
* [Using Bayesian Update API to Update Objects](using_bayes_update_api.md)
* [Adding Custom Predicates](adding_predicates.md)

