#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include <mutex>

#include "pipes.h"

using pfms::commands::UAV;

class UAVComms
{
public:
UAVComms(ros::NodeHandle nh) :
    nh_(nh),seq_(0)
{
    pubCmd_ = nh_.advertise<geometry_msgs::Twist>("/drone/cmd_vel",5,false);
    pubTakeOff_ = nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1024);
    pubLand_ = nh_.advertise<std_msgs::Empty>("/drone/land", 1024);

    pipes = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
}

~UAVComms(){
    delete pipes;
}

void run(void){

    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    pubTakeOff_.publish(std_msgs::Empty());
    ROS_INFO("Taking Off...");

    ros::Rate rate_limiter(20.0);

    while (ros::ok()){

      UAV uav;
      bool OK = pipes->readCommand(uav);
      if(OK){
          ROS_DEBUG_STREAM( "seq " <<  uav.seq << "\n" <<
                           "turn_l_r " <<  uav.turn_l_r << "\n" <<
                           "move_l_r " <<  uav.move_l_r << "\n" <<
                           "move_u_d " <<  uav.move_u_d << "\n" <<
                           "move_f_b " <<  uav.move_f_b);

          if(uav.seq!=seq_){

              geometry_msgs::Twist  twist_msg;
              twist_msg.linear.x = uav.move_f_b;
              twist_msg.linear.y = uav.move_l_r;
              twist_msg.linear.z = uav.move_u_d ;
              twist_msg.angular.x = 0.0; // flag for preventing hovering
              twist_msg.angular.y = 0.0;
              twist_msg.angular.z = uav.turn_l_r;

              pubCmd_.publish(twist_msg);

              seq_=uav.seq;
          }
      }
      else {
          ROS_WARN("No UAV message received in 5 seconds");
          delete pipes;
          pipes = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      rate_limiter.sleep();
    }
}

void land(){
    pubLand_.publish(std_msgs::Empty());
    ROS_INFO("Landing ...");
}

private:
    ros::NodeHandle nh_;
    ros::Publisher pubCmd_, pubTakeOff_, pubLand_ ;
    unsigned long seq_;
    Pipes* pipes;
};



/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_comms");
  ros::NodeHandle nh;
  std::shared_ptr<UAVComms> uav_comms(new UAVComms(nh));

  std::thread run_thread(&UAVComms::run,uav_comms);

  ros::spin();

  ros::shutdown();

  run_thread.join();

  return 0;
}
