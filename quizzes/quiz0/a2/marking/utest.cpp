#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../sample.h"

using namespace std;


TEST (ClassTest, CreateObject) {
    Sample sample(5.0);
    EXPECT_EQ(sample.readValue(), 5.0);
}

//2) TASK: Implement ALL the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h).
// Your code will not compile until the functions are implemented.
TEST (ClassTest, FunctionalityObject) {
    Sample sample(5.0);
    EXPECT_EQ(sample.readValue(), 5.0);
    sample.setValue(-5.0);
    EXPECT_EQ(sample.readValue(), -5.0);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
