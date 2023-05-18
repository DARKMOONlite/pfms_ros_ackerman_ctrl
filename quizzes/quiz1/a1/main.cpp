#include <iostream> // Includes std::cout and friends so we can output to console
#include "person.h"
#include "processing.h"

int main (void) {
    //! Your main is not tested, we test classes you develop
    //! However, to assist your own development you can
    //! use the main to create objects, use functions of them
    //! and check the output
  //? Task 1: 
    Person Bob("Bob",22,false);
  std::cout << "Age of " << Bob.getName() << ":" << Bob.getAge() << std::endl;
  std::cout << "is " << Bob.getName() << " Vaccinated: " << Bob.getVaccinatedStatus() <<std::endl;
  Bob.vaccinate(); 
  std::cout << "is " << Bob.getName() << " Vaccinated: " << Bob.getVaccinatedStatus() <<std::endl;
  
    //! TASK 3
    //! Create a vector of Person's (crowd)
    //!
    //! --  Example is below 5 elements --
    //! Alice, 32 year old, vaccinated
    //! Bob, 62 year old male, unvaccinated
    //! Carol, 72 year old female, vaccinated
    //! John, 82 year old male, unvaccinated
    //! Karen, 42 year old female, vaccinated
    Person Alice("Alice", 32, true);
    Person Bob2("Bob", 62, false);
    Person Carol("Carol",72,true);
    Person John("John", 82, false);
    Person Karen("Karen", 42, true);
    Person Alen("Alen", 82 , true);
    std::vector<Person> crowd;
    crowd.push_back(Alice);
    crowd.push_back(Bob2);
    crowd.push_back(Carol);
    crowd.push_back(John);
    crowd.push_back(Karen);
    crowd.push_back(Alen);

    //! Call
   std::vector<Person> oldest = oldestPerson(crowd);
   std::cout << "Oldest Person are: ";
   for(int it = 0; it < oldest.size(); it++){
      std::cout << oldest.at(it).getName() << "   ";

   }
    //! TASK 4
    //!
    //! specify and age cutoff and call
    //! std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff);
    unsigned int ageCutoff = 50;
    std::vector<Person> requires_Vaccine;
    requires_Vaccine = eligibleForVaccine(crowd, ageCutoff);
    for(int i=0; i<requires_Vaccine.size(); i++){
      std::cout << "These People Require Vaccines: "<<requires_Vaccine[i].getName() << std::endl;
   }


  return(0);
}
