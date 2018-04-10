# Adding Custom Predicates

There are two types of predicates exist in Violet: unary and binary predicates. Unary predicates affect single object and denote spatio-temporal features like being in the field of view of the camera or being on the table. Binary predicates denote relations between two objects such as being near to or  being on top of another object.

All predicates must extend `Predicate` class and apropriate `check` method for run-time evaluation. The constructor of `Predicate` class takes two arguments namely: predicate name and predicate type. The `name` argument is published when the predicate instance is added to Knowledge Base. `type` argument determines the predicate type and which version of check method to be used in automatic evaluation. Unary predicates should override unary version of the `check` method and binary predicates vice versa. Overriding the wrong version of a predicate will result in an exception to be thrown.

Two example  implementations of unary and binary predicates are below:

### Unary Predicate
The class declaration and instance declaration for referencing of an example unary predicate is below:
```c++
namespace violet {
namespace predicates {
class UnaryPredicate : public Predicate
{
    //Private members used to calculate predicate
public:
    UnaryPredicate(); // Constructor
    virtual bool check(const ObjectInfo &object); //Override unary version of check method
};
extern UnaryPredicate unary_instance;
}
}
```
The implementation of unary predicate is like below:

```c++
UnaryPredicate::UnaryPredicate() : Predicate("unary", PREDICATE_UNARY)
{
    PredicateManager::registerPredicate(*this); // Register this instance to automatic checking for all objects
}

bool UnaryPredicate::check(const ObjectInfo &object)
{
     if(/*object satisfies my property*/) {
         return true;
     }
     
     return false;
}

UnaryPredicate unary_instance;
```

### Binary Predicate
Binary predicate is implemented similar to unary predicate. The direction of the evalutation is from `object1` to `object2`.
```c++
namespace violet {
namespace predicates {
class BinaryPredicate : public Predicate
{
    //Private members used to calculate predicate
public:
    BinaryPredicate(); // Constructor
    virtual bool check(const ObjectInfo &object1, const ObjectInfo &object2); //Override unary version of check method
};
extern BinaryPredicate binary_instance;
}
}
```
The implementation of unary predicate is like below:

```c++
namespace violet {
namespace predicates {
BinaryPredicate::BinaryPredicate() : Predicate("binary", PREDICATE_BINARY)
{
    PredicateManager::registerPredicate(*this); // Register this instance to automatic checking for all objects
}

bool BinaryPredicate::check(const ObjectInfo &object1, const ObjectInfo &object2)
{
     if(/*object satisfies my property*/) {
         return true;
     }
     
     return false;
}

}
}
BinaryPredicate binary_instance;
```