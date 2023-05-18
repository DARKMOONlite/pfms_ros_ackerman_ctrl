#include "carmock.h"

MockCar::MockCar(std::string make, std::string model,double height, double width,
                 double horsePower, double dragCoefficient, double weigh, double mockOdo):
    Car(make, model,height, width,horsePower, dragCoefficient, weigh) {
  //Extra constructor for easy mocking
  mockOdo_ = mockOdo;
}


double MockCar::getOdometry() {
  return mockOdo_;
}
