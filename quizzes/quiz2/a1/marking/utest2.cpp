#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>
#include "gmock/gmock.h"

//header files needed from our libraries
#include "../analysis.h"
//Preted car to inject odometry
#include "mock/carmock.h"
using namespace std;


TEST (AnalysisTest, SortOdo) {

    std::vector<CarInterface*> cars;
    cars.push_back(new MockCar("merc", "c180",1.45,1.77,143,0.29,1200,200.1));
    cars.push_back(new MockCar("bugatti", "veyron",1.19,2.00,1200,0.35,2200,210.2));
    cars.push_back(new MockCar("toyota", "yaris",1.19,1.87,420,0.30,1190,210.0));

    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->sortByOdometry();

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),0);
    EXPECT_EQ(order.at(1),2);
    EXPECT_EQ(order.at(2),1);
}

TEST (AnalysisTest, zeroTopZeroRace) {
    std::vector<CarInterface*> cars;
    cars.push_back(new Car("merc", "c180",1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron",1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris",1.19,1.87,420,0.30,1190));

    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars));

    std::vector<unsigned int> order = analysisPtr->zeroTopZeroRace();

    ASSERT_EQ(order.size(),cars.size());
    EXPECT_EQ(order.at(0),1);
    EXPECT_EQ(order.at(1),2);
    EXPECT_EQ(order.at(2),0);

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
