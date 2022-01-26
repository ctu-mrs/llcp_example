#include <ros/package.h>
#include <ros/ros.h>
#include <mrs_msgs/Llcp.h>

#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <example_msgs.h>

namespace llcp_example
{

/* class LlcpExample //{ */

class LlcpExample : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::Timer send_timer_;

  void callbackSendTimer(const ros::TimerEvent &event);
  void callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg);

  ros::NodeHandle nh_;

  ros::Publisher  llcp_publisher_;
  ros::Subscriber llcp_subscriber_;

  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void LlcpExample::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  /* nh_.param("uav_name", uav_name_, std::string("uav")); */
  /* nh_.param("portname", portname_, std::string("/dev/ttyUSB0")); */
  /* nh_.param("baudrate", baudrate_, 115200); */
  /* nh_.param("publish_bad_checksum", publish_bad_checksum, false); */
  /* nh_.param("use_timeout", use_timeout, true); */
  /* nh_.param("serial_rate", serial_rate_, 5000); */
  /* nh_.param("serial_buffer_size", serial_buffer_size_, 1024); */

  // Publishers
  llcp_publisher_ = nh_.advertise<mrs_msgs::Llcp>("llcp_out", 1);

  llcp_subscriber_ = nh_.subscribe("llcp_in", 10, &LlcpExample::callbackReceiveMessage, this, ros::TransportHints().tcpNoDelay());

  // Output loaded parameters to console for double checking
  /* ROS_INFO_THROTTLE(1.0, "[%s] test test:", ros::this_node::getName().c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str()); */
  /* ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), baudrate_); */
  /* ROS_INFO_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName().c_str() << "] publishing messages with wrong checksum: " << publish_bad_checksum); */

  /* connectToSensor(); */

  /* serial_timer_     = nh_.createTimer(ros::Rate(serial_rate_), &LlcpExample::callbackSerialTimer, this); */
  send_timer_ = nh_.createTimer(ros::Rate(1), &LlcpExample::callbackSendTimer, this);

  is_initialized_ = true;
}
//}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackReceiveMessage() //{ */

void LlcpExample::callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  std::vector<uint8_t> payload_vec  = msg->payload;
  uint8_t              payload_size = payload_vec.size();
  uint8_t              payload_array[payload_size];
  std::copy(payload_vec.begin(), payload_vec.end(), payload_array);

  ROS_INFO_STREAM("[LlcpExample]: got id: " << msg->id << " hb: " << example_heartbeat_msg::id << " data: " << example_data_msg::id);
  example_data_msg tmp;
  switch (msg->id) {
    case tmp.id: {
      ROS_INFO("[LlcpExample]: DATA");
      break;
    }
    case example_heartbeat_msg::id: {
      ROS_INFO("[LlcpExample]: HB");
      break;
    }
    default: {
      ROS_INFO("[LlcpExample]: default");
      break;
    }
  }


  /*     break; */
  /*   } */
  /* } */

  example_data_msg *received_msg = (example_data_msg *)payload_array;
  ROS_INFO_STREAM("[LlcpExample]: Received llcp message -> data1: " << received_msg->data1 << ", data2: " << received_msg->data2
                                                                    << ", data3: " << received_msg->data3);
}

//}

/* callbackSendTimer() //{ */

void LlcpExample::callbackSendTimer(const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  example_data_msg msg_out;

  msg_out.data1 = 66;
  msg_out.data2 = 4532;
  msg_out.data3 = 644532;

  mrs_msgs::Llcp llcp_msg;
  llcp_msg.id = msg_out.id;

  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  ROS_INFO_STREAM("[LlcpExample]: Sending a llcp example message -> data1: " << msg_out.data1 << ", data2: " << msg_out.data2 << ", data3: " << msg_out.data3);
  llcp_publisher_.publish(llcp_msg);
}

//}

// | ------------------------ routines ------------------------ |


}  // namespace llcp_example

PLUGINLIB_EXPORT_CLASS(llcp_example::LlcpExample, nodelet::Nodelet);
