#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>

//header files needed from our libraries
#include "../src/analysis.h"
using namespace std;


TEST (AnalysisTest, CountCharacters) {


  unsigned int count = analysis::countCharacters("Hello class 41012");
  EXPECT_EQ(count,17);

}
TEST (AnalysisTest, getNumber) {


   int num = analysis::getNumber("Hello class 41012");
  EXPECT_EQ(num,41012);

}

TEST (AnalysisTest, multipleNumber) {


   int num = analysis::getNumber("C3PO & R2D2");
  EXPECT_EQ(num,3);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
