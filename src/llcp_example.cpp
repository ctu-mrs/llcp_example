#include <ros/package.h>
#include <ros/ros.h>
#include <mrs_msgs/Llcp.h>

#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "../arduino_example/example_msgs.h"

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

  uint8_t  my_data1_uint8  = 10;
  uint32_t my_data2_uint32 = 1000;
  float   my_data3_float    = 142.46;

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
  uint8_t              payload_size = msg->payload.size();
  uint8_t              payload_array[payload_size];
  std::copy(msg->payload.begin(), msg->payload.end(), payload_array);

  switch (msg->id) {
    case data_msg::id: {

      data_msg *received_msg = (data_msg *)payload_array;

      ROS_INFO_STREAM("[LlcpExample]: Received data message -> data1_uint8: " << unsigned(received_msg->data1_uint8) << ", data2_uint32: " << received_msg->data2_uint32
                                                                              << ", data3_float: " << received_msg->data3_float);
      break;
    }
    case heartbeat_msg::id: {
      ROS_INFO("[LlcpExample]: Received Heartbeat message");
      break;
    }
    default: {
      ROS_INFO("[LlcpExample]: default");
      break;
    }
  }
}

//}

/* callbackSendTimer() //{ */

void LlcpExample::callbackSendTimer(const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  data_msg msg_out;

  msg_out.data1_uint8  = my_data1_uint8;
  msg_out.data2_uint32 = my_data2_uint32;
  msg_out.data3_float = my_data3_float;

  my_data1_uint8++;
  my_data2_uint32 += 10;
  my_data3_float += 10.5;

  mrs_msgs::Llcp llcp_msg;
  llcp_msg.id = msg_out.id;

  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  ROS_INFO_STREAM("[LlcpExample]: Sending data message -> data1_uint8: " << unsigned(msg_out.data1_uint8) << ", data2_uint32: " << msg_out.data2_uint32
                                                                         << ", data3_float: " << msg_out.data3_float);
  llcp_publisher_.publish(llcp_msg);
}

//}

// | ------------------------ routines ------------------------ |


}  // namespace llcp_example

PLUGINLIB_EXPORT_CLASS(llcp_example::LlcpExample, nodelet::Nodelet);
