#ifndef CONTAINER_OPS_H
#define CONTAINER_OPS_H

#include <deque>


/**
 * @brief accepts a container and modifies it by adding user specified numbers of elements to the begining of the
 * container.
 * @param container to be modified
 * @param num_vales number of identical elements to be added
 * @param elelemt the value to be added
 */
void populateContainer(std::deque<double>& container, unsigned int num_values, double element);

void bubbleSortContainer( std::deque<double>& container);

#endif // CONTAINER_OPS_H
