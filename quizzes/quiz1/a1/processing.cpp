#include "processing.h"
// You can add additional headers needed here, such as
#include <limits>
#include <algorithm>
//3) TASK Implement a function that returns the `Person` who is the oldest member of the `crowd`.
std::vector<Person> oldestPerson(std::vector<Person> crowd){
    int Comparison = 0;
    int Largest_Index = 1;
    for(int i=0; i < crowd.size(); i++){
        if(crowd[i].getAge()>= Comparison){
            Comparison = crowd[i].getAge();
        }
    }
    std::vector<Person> result;
   for(int i=0; i < crowd.size(); i++){
       if(crowd[i].getAge()>=Comparison)
        result.push_back(crowd[i]);
   }
    return(result);

    
}


//4) TASK Implement a function that returns the `people` from the `crowd` that need to be vaccinated.
//Criteria is that they are older then the specified `ageCuttoff` and have not been previously vaccinated.
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff){
//for(std::vector<Person>::iterator it = crowd.begin(); it != crowd.end(); it++){ //figure out how to make this work
 std::vector<Person> unvaccinated_Individuals;
 
 for(int i=0; i < crowd.size(); i++){
   if(crowd[i].getAge()>=ageCutoff && crowd.at(i).getVaccinatedStatus() == false){//if the person is old enough and unvaccinated then push back into vector
       unvaccinated_Individuals.push_back(crowd.at(i));
       }
       
       

}
    return(unvaccinated_Individuals);
}
