#ifndef QUADCOPTER_TEST_H
#define QUADCOPTER_TEST_H

#include "gtest/gtest.h"
#include "pipes.h"

// The below is to setup our Unit Tests, Not required to understand, skip to wheer tests start
class QuadcopterTest : public testing::Test {
 protected:
  // Per-test-suite set-up.
  // Called before the first test in this test suite.
  // Can be omitted if not needed.
  //static void SetUpTestSuite() {
  //  // Avoid reallocating static objects if called in subclasses of FooTest.
  //    if (pipesFakeOdo_ == nullptr) {
  //      pipesFakeOdo_ = new Pipes(8,"/fake_odo_buffer_seg","/fake_odo_buffer_wsem","/fake_odo_buffer_rsem");
  //    }
  //}

  static void SetUpTestCase() {
    // Avoid reallocating static objects if called in subclasses of FooTest.
    if (pipesFakeOdo_ == nullptr) {
      pipesFakeOdo_ = new Pipes(8,"/fake_odo_buffer_seg","/fake_odo_buffer_wsem","/fake_odo_buffer_rsem");
    }
  }

  // Per-test-suite tear-down.
  // Called after the last test in this test suite.
  // Can be omitted if not needed.
  //static void TearDownTestSuite() {
  //    if (pipesFakeOdo_ != nullptr){
  //      delete pipesFakeOdo_;
  //      pipesFakeOdo_ = nullptr;
  //    }
  //}

  static void TearDownTestCase() {
      if (pipesFakeOdo_ != nullptr){
        delete pipesFakeOdo_;
        pipesFakeOdo_ = nullptr;
      }
  }

  static Pipes* pipesFakeOdo_;

};

Pipes* QuadcopterTest::pipesFakeOdo_ = nullptr;

#endif // QUADCOPTER_TEST_H
