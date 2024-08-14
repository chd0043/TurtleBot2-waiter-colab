#include <ros/ros.h>
#include <std_msgs/Bool.h>

class coordinator
{
public:
  coordinator() {
      request_pub_[0] = n_.advertise<std_msgs::Bool>("/table0/sim_coordinator",1000);
      request_pub_[1] = n_.advertise<std_msgs::Bool>("/table1/sim_coordinator",1000);
      request_pub_[2] = n_.advertise<std_msgs::Bool>("/table2/sim_coordinator",1000);
      request_pub_[3] = n_.advertise<std_msgs::Bool>("/table3/sim_coordinator",1000);
      request_pub_[4] = n_.advertise<std_msgs::Bool>("/table4/sim_coordinator",1000);
  }
  virtual ~coordinator() {}

  void spin()
  {
    int counter(0);
    while ( ros::ok() )
    {
      if ( counter < 50 )
      {
        testSequence1();
        counter++;
        ROS_INFO("Counter: %i", counter);
      }
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher request_pub_[4];

  void forceRequest(int table_num_id)
  {
    std_msgs::Bool msg;
    msg.data = true;

    request_pub_[table_num_id].publish(msg);
    ros::Duration(0.5).sleep();
  }

  void testSequence1(float timer=40.0)
  {
    waitTime(timer);
    forceRequest(0);
    forceRequest(1);
    forceRequest(2);
    forceRequest(3);
  }

  bool waitTime(double time_preset)
  {
    double timeout(0);
    while (timeout < time_preset && ros::ok())
    {
        ros::Duration(0.1).sleep(); //sample_time_
        timeout +=0.1;
        //ros::spinOnce();
    }
    return(true);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinator");
  ROS_INFO("Coordinator on.");

  coordinator object;
  object.spin();



  return (0);
}



