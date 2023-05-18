Quiz 1
======

Part A1
------

1) TASK
The `Person` class is missing a special member function. This function needs to enable creating an object of `Person` class with the `name` , `age` and `vaccinated` status initialised with values supplied by user of the class. You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as well as implement this function in [implementation file of Person class](./a1/person.cpp).
2) TASK
Implement the method `vaccinate` in `Person` class. This function returns a `bool` indicating if a person can be given a vaccine. The person can be given a vaccine, if they are not already vaccinated.  When a person is given a vaccine, their `vaccinated` status should change. 


3) TASK
Implement [oldestPerson](./a1/processing.h) function that returns the `Person` who is the oldest person(s) of the `crowd`. Create a `crowd` using a STL vector container of people, populate it with 5 people. To test you can use details of the people are in the [main](./a1/main.cpp) or improvise.

4) TASK
Implement [eligibleForVaccine](./a1/processing.h) function that returns the `people` from the `crowd` that need to be vaccinated. Criteria is that they are older then the specified `ageCuttoff` and have not been previously vaccinated. The age cut-off is inclusive of the age specified (an example is serving liquor to 18+, 18 and above).

Part A2
------

1. TASKS

* Modify the `Rectangle` class so it inherits from the base class of shape [shape](./a2/shape.h). 

* Correct the missing access specifiers of base class [shape](./b2/shape.h) so that the implementation of the [`Rectangle` constructor](./a2/rectangle.cpp) can still access the required member variable of `Shape` but have it restricted to users of a `Rectangle` object.

* Enable the  `Rectangle` class to have another function that enables the `Rectangle` to on creation have `width` , `height`  initialised with values supplied by user of the class and sets the description on creation of the class to be either `square` or `rectangle` depending on the `width` and  `heigth` supplied by the user. 

  **NOTE: The user should not specify the description, only the width and height **

2. TASK
    Implement function `void removeLargerThanArea(std::vector<Shape*> &shapes, double limit);` which is in [processing.h](./a2/processing.h). The function removes from the container of `shapes` any shape that has an area greater than the `limit`.  
    In your main create a number of rectangles and check the function works as specified. 

  **NOTE: Check the function declaration,   What do we need to pass to the function? (look at  Week 03 : Ex03 solution discussion)** 

