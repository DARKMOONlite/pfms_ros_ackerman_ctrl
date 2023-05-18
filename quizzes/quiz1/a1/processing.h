#ifndef PROCESSING_H
#define PROCESSING_H
#include "person.h"
#include <vector>

/**
 * @brief Returns the `Person` who is the oldest person(s) of the `crowd`.
 * @param crowd vector of Person
 * @return The vector of oldest people
 */
std::vector<Person> oldestPerson(std::vector<Person> crowd);


/**
 * @brief Function that returns person(s) that need to be vaccinated : older then the specified `ageCuttoff` and have not been previously vaccinated
 * @param crowd vector of Person
 * @param ageCutoff the age above which people are eligible
 * @return The vector of people eligible for vaccination
 */
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff);


#endif // PROCESSING_H
