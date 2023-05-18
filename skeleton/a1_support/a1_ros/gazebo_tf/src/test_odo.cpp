#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include "tf/transform_datatypes.h"
#include "pipes.h"
#include <string>

class TestOdo
{
public:
TestOdo(ros::NodeHandle nh) :
    nh_(nh),uavSeq_(0),ugvSeq_(0),odoUAVactive_(false),odoUGVactive_(false)
{
    pipesOdoUGV_ = new Pipes(8,"/ugv_odo_buffer_seg","/ugv_odo_buffer_wsem","/ugv_odo_buffer_rsem");
    pipesOdoUAV_ = new Pipes(8,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem");
    pipesFakeOdo_ = new Pipes(8,"/fake_odo_buffer_seg","/fake_odo_buffer_wsem","/fake_odo_buffer_rsem",true);
    // UAV default values
    odoUAV_.seq = uavSeq_++;
    odoUAV_.x = 0;
    odoUAV_.y = 0;
    odoUAV_.yaw = 0;
    odoUAV_.vx = 0;
    odoUAV_.vy = 0;

    // UGV default values
    odoUGV_.seq = ugvSeq_++;
    odoUGV_.x = 0;
    odoUGV_.y = 2;
    odoUGV_.yaw = 0;
    odoUGV_.vx = 0;
    odoUGV_.vy = 0;

    std::this_thread::sleep_for (std::chrono::milliseconds(50));

    ROS_INFO_STREAM("Starting to listen to fake data");

    //Not used
//    ros::NodeHandle pnh("~"); // Create private node handle
//    link_name_="SharedMemoryROS"; // Default Name
//    pnh.getParam("link_name", link_name_);
//    ROS_INFO_STREAM("The connection to ros on _link_name:" << link_name_);
}

~TestOdo(){
    delete pipesOdoUGV_;
    delete pipesOdoUAV_;
    delete pipesFakeOdo_;
}


void runUAVOdo(void){

    ros::Rate rate_limiter(50.0);

    while (ros::ok()){
      if(odoUAVactive_){
        bool OK = pipesOdoUAV_->writeCommand(odoUAV_);
        if(!OK){
          ROS_WARN_STREAM("No UAV Odo requested in 5 seconds");
          delete pipesOdoUAV_;
          pipesOdoUAV_ = new Pipes(7,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      rate_limiter.sleep();
    }
}

void runUGVOdo(void){

    ros::Rate rate_limiter(50.0);

    while (ros::ok()){
      if(odoUGVactive_){
        bool OK = pipesOdoUGV_->writeCommand(odoUGV_);
        if(!OK){
          ROS_WARN_STREAM("No UGV Odo requested in 5 seconds");
          delete pipesOdoUGV_;
          pipesOdoUGV_ = new Pipes(7,"/ugv_odo_buffer_seg","/ugv_odo_buffer_wsem","/ugv_odo_buffer_rsem");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      rate_limiter.sleep();
    }
}

void comms(void){

    ROS_INFO("Ready to recieve location");
    while (ros::ok()){

        pfms::nav_msgs::Odometry odo =  pipesFakeOdo_->getOdo();
        ROS_INFO_STREAM("Set location veh [1/2] " << odo.seq << " odo  " << odo.x << " " <<  odo.y);
        switch (odo.seq) {
          case 1:
          {
            odoUGV_.seq = ugvSeq_++;
            odoUGV_.x = odo.x;
            odoUGV_.y = odo.y;
            odoUGV_.yaw = odo.yaw;
            odoUGV_.vx = 0;
            odoUGV_.vy = 0;
            ROS_INFO_STREAM("Sending Fake UGV ");
            odoUGVactive_=true;
            break;
          }
          case 2:
          {
            odoUAV_.seq = uavSeq_++;
            odoUAV_.x = odo.x;
            odoUAV_.y = odo.y;
            odoUAV_.yaw = odo.yaw;
            odoUAV_.vx = 0;
            odoUAV_.vy = 0;
            ROS_INFO_STREAM("Sending Fake UAV ");
            odoUAVactive_=true;
            break;
          }
          default:
          {
            ROS_WARN_STREAM("The seq number corresponds to UAV/UGV for this testing and " << odo.seq << " sent, can not be used");
            break;
          }
        }//switch
    }



}


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    Pipes* pipesOdoUAV_;
    Pipes* pipesOdoUGV_;
    Pipes* pipesFakeOdo_;
    unsigned long uavSeq_;
    unsigned long ugvSeq_;
    pfms::nav_msgs::Odometry odoUAV_;
    pfms::nav_msgs::Odometry odoUGV_;
    //std::string link_name_;
    bool odoUAVactive_;
    bool odoUGVactive_;

};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_odo");

  ros::NodeHandle nh;

  std::shared_ptr<TestOdo> test_odo(new TestOdo(nh));

  std::thread uav_odo_thread(&TestOdo::runUAVOdo,test_odo);
  std::thread ugv_odo_thread(&TestOdo::runUGVOdo,test_odo);
  std::thread comms_thread(&TestOdo::comms,test_odo);

  ros::spin();

  ros::shutdown();

  uav_odo_thread.join();
  ugv_odo_thread.join();
  comms_thread.join();

  return 0;
}

