#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include "pipes.h"

using pfms::commands::UGV;

class UGVComms
{
public:
UGVComms(ros::NodeHandle nh) :
    nh_(nh),seq_(0),steering_(0)
{
    throttle_pub = nh_.advertise<std_msgs::Float64>("/orange/throttle_cmd", 10, false);
    brake_pub = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd", 10, false);
    steering_pub = nh_.advertise<std_msgs::Float64>("/orange/steering_cmd", 10, false);
    pipes = new Pipes(pipes::Type::CONSUMER, 5,"/ugv_buffer",false);
}

~UGVComms(){
    delete pipes;
}

void run(void){

    ros::Rate rate_limiter(20.0);

    while (ros::ok()){

      UGV ugv;
      if(pipes->readCommand(ugv)){
          ROS_DEBUG_STREAM( "seq brake/steering/throttle " <<  ugv.seq << " "
                           << ugv.brake << " " << ugv.steering << " " << ugv.throttle);

          if(ugv.seq!=seq_){

              std_msgs::Float64 msg_brake, msg_throttle, msg_steering;
              msg_brake.data = ugv.brake;
              brake_pub.publish(msg_brake);

              msg_throttle.data = ugv.throttle;
              throttle_pub.publish(msg_throttle);

              //Let's only send steering if it has changed from previous value.
//              ROS_DEBUG_STREAM("steering prev/now " << steering_  << "/" << ugv.steering <<
//                              "diff:" << fabs(steering_-ugv.steering));
              if(fabs(steering_-ugv.steering)>0.01){
                  msg_steering.data = ugv.steering;
                  steering_pub.publish(msg_steering);
                  steering_=ugv.steering;
              }

              ros::spinOnce();
              seq_=ugv.seq;
          }
      }
      else {
          ROS_WARN("No UGV message received in 5 seconds");
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      rate_limiter.sleep();
    }
}

private:
    ros::NodeHandle nh_;
    ros::Publisher throttle_pub ,brake_pub ,steering_pub;
    unsigned long seq_;
    double steering_;
    Pipes* pipes;
};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ugv_comms");

  ros::NodeHandle nh;

  std::shared_ptr<UGVComms> ugv_comms(new UGVComms(nh));

  std::thread run_thread(&UGVComms::run,ugv_comms);

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  run_thread.join();

  return 0;
}
