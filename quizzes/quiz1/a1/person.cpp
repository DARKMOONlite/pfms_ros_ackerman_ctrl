#include "person.h"

//    1) TASK The `Person` class is missing a special member function. This function needs to enable crteating an object of
//    `Person` class with the `name` , `age` and `vacinated` initialised with values supplied by user of the class.
//    You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as
//    well as implement this function in the [implementation file of Person class](./a1/person.cpp).
Person::Person(std::string name, int age, bool vaccinated){
  name_ = name;
  age_ = age;
  vaccinated_ = vaccinated;


}

std::string Person::getName(void) {
  return name_;
}

unsigned int Person::getAge(void) {
  return age_;
}


bool Person::getVaccinatedStatus(void) {
  return vaccinated_;
}

//2) TASK
// Implement the method `vaccinate` in `Person` class.
// This function adminiretruns a `bool` indicating if a person can be given a vaccine.
// The person can be given a vaccine, if they are not already vaccinated.
//
//When a person is given a vaccine, their `vaccinated` status should change.
bool Person::vaccinate(void){
  if (getVaccinatedStatus()){ // if the person is already vaccinated then return false.
    return false;
  }
  else{ //if the person is not vaccineated then return true to indicate that they can be vaccinated
  vaccinated_ = true;
  return true;
  }

}
