#include "databuffer.h"
#include <random>
#include <chrono>
#include <thread>

DataBuffer::DataBuffer(){
    val_.clear();
}

bool DataBuffer::removeValues(double min, double max){
    bool removed=false;
    // We use unique lock rather than locking and unlocking the mutex directly
    // http://www.cplusplus.com/reference/mutex/unique_lock/
    std::unique_lock<std::mutex> lck (mtx_);

    auto it = val_.begin();

    while ( it != val_.end()) {
        if (*it < min || *it > max) {
            val_.erase(it);
            removed=true;
        } else {
            it++;
        }
    }
    return removed;
}

void DataBuffer::addValue(double value){
  std::unique_lock<std::mutex> lck (mtx_);
  val_.push_back(value);
}

void DataBuffer::trimLength(unsigned int value){

  std::unique_lock<std::mutex> lck (mtx_);

  while (val_.size()>value) {
      val_.erase(val_.begin());
  }
}

