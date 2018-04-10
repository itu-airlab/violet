# Adding Your Own Custom Input Source

The class hierarchy of Violet provides flexibility to how to interpret the data generated from your inpur source. An input source could be anything like a computer vision algorithm's output, an information source like a node that publishes marked objects in a map  or a node that publishes location of a gripper with an object in it. A generic message format is provided with Violet; what information will be put there and how it will be interpreted is left to developer.

## Writing the Input Source Interpreter

After designing your source node, decide on how to encode your message as ```violet_msgs/DetectionInfo```. The next step is designing an input source interpreter that extends ```violet::InputSource``` class. An empty example of an input source interpreter looks like this:

```c++
namespace violet {
namespace inputsources {

class MyInputSourceInterpreter : public InputSource
{
public:
    MyInputSourceInterpreter(const std::string source_name);
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr &msg);
};

MyInputSourceInterpreter::MyInputSourceInterpreter(const std::string source_name) : InputSource(source_name),
{

    /*
     * Confusion matrix for adding/updating object's confidence probability:
     * Some properities can be updated without the need of this matrix. However
     * it's used for updating confidence of the existance of an object and generally
     * used by an input source. The values are the probabilities of: 
     *   - Detecting an object when it really existing
     *   - Not detecting an existing object in the scene
     *   - Detecting an object when it doesn't exist in the scene
     *   - Not detecting an object.
     *
     * Generally first two values are sufficient to implement an input source since,
     * update method of main storage class, ObjectInfo::increaseConfidence, uses
     * However if you might want to decrease the probability of an object in the
     * input source interpreter then the last two values can be used for decreasing
     * the object probabilities for calling  ObjectInfo::decreaseConfidence function.
     */
    double confidence_defaults[4] = {0.55, 0.45, 0.4, 0.6};

    for(int i = 0; i < 4; ++i) {
        _confidence_confusion[i%2][i/2] = confidence_defaults[i];
    }

}

void MyInputSourceInterpreter::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{
    // Parse the message and add/update objects
}

DEFINE_INPUT_SOURCE(MyInputSource, MyInputSourceInterpreter)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
```

At the end of a source file use ```DEFINE_INPUT_SOURCE``` macro to register the input source with the name ```MyInputSource``` and the interpreter class ```MyInputSourceInterpreter```. It's possible to reuse the same interpreter with multiple input sources. For example, if your robot has two grippers registering two input sources is possible like:

```
DEFINE_INPUT_SOURCE(Gripper1, GripperInterpreter)
DEFINE_INPUT_SOURCE(Gripper2, GripperInterpreter)
```
The constrtuctor of your input source interpreter will be called with the first parameter of `DEFINE_INPUT_SOURCE` macro.

When it's ready put your ```.cpp``` file into ```violet/src/inputsources``` directory and recompile the violet package using ```catkin_make```.

## Communicating with Violet

To make Violet to spawn your interpreter at runtime an input source node should send a listener registration request it from  ```/violet/register_source``` service in the message format ```violet_srvs/RegisterSource.srv```.

The request fields of   ```violet_srvs/RegisterSource.srv``` are below:
```
string topic_name
string source_algorithm_name
```
```topic_name``` is the topic you are planning to publish messages and ```source_algorithm_name``` is the name of your inpur source defined in the interpreter source file, e.g Gripper1, MyInputSource

After a successful registration the ```bool success``` field of response will return ```true``` then the input node can start communicating with Violet by  publishing DetectionInfo messages to the denoted topic in the request.

