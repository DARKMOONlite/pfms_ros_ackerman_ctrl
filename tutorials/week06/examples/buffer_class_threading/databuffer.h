#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <string>
#include <vector>
#include <mutex>

using std::vector;

/*!
 * \brief     DataBuffer Class
 * \details
 * This class demonstrates how to create thread safe class, the enter user now does not have to deal
 * with mutexe's as the functions internally handle this.
 * \author    Alen Alempijevic
 * \version   1.00-2
 * \date      2020-09-10
 * \pre       none
 * \bug       none reported as of 2020-04-11
 * \warning
 */
class DataBuffer
{
public:
    DataBuffer();

    /**
    Member function to remove values from internal container
    @param min remove values below this minimum value
    @param max remove values above this maximum value
    @return if a value has been removed
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    bool removeValues(double min, double max);
    /**
    Member function to add values to internal container
    @param value value to be added
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    void addValue(double value);
    /**
    Member function to trim internal container to specified size
    @param size size to be trimmed to
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    void trimLength(unsigned int size);

private:
    std::string name_;
    vector<double> val_;
    std::mutex mtx_;
};

#endif // DATABUFFER_H
