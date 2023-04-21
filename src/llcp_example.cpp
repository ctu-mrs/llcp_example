#include <ros/package.h>
#include <ros/ros.h>
#include <mrs_msgs/Llcp.h>
#include <mrs_msgs/SetTrigger.h>
#include <mrs_msgs/SetServo.h>

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
  void callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg);
  bool callbackSetServo(mrs_msgs::SetServo::Request &req, mrs_msgs::SetServo::Response &res);
  bool callbackSetTrigger(mrs_msgs::SetTrigger::Request &req, mrs_msgs::SetTrigger::Response &res);

  ros::NodeHandle nh_;

  ros::Publisher  llcp_publisher_;
  ros::Subscriber llcp_subscriber_;

  ros::ServiceServer service_server_set_servo_;
  ros::ServiceServer service_server_set_trigger_;

  bool is_initialized_ = false;
};

//}

/* onInit() //{ */

void LlcpExample::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // Publishers
  llcp_publisher_ = nh_.advertise<mrs_msgs::Llcp>("llcp_out", 1);

  llcp_subscriber_ = nh_.subscribe("llcp_in", 10, &LlcpExample::callbackReceiveMessage, this, ros::TransportHints().tcpNoDelay());

  service_server_set_servo_   = nh_.advertiseService("set_servo_in", &LlcpExample::callbackSetServo, this);
  service_server_set_trigger_ = nh_.advertiseService("set_trigger_in", &LlcpExample::callbackSetTrigger, this);

  is_initialized_ = true;
}
//}

//}

// | ------------------------ callbacks ------------------------ |


/* //{ callbackSetServo() */

bool LlcpExample::callbackSetServo(mrs_msgs::SetServo::Request &req, mrs_msgs::SetServo::Response &res) {

  if (!is_initialized_) {
    res.success = false;
    return false;
  }

  servo_msg msg_out;

  msg_out.id         = SERVO_MSG_ID;
  msg_out.servo1_pos = req.servo1_val;
  msg_out.servo2_pos = req.servo2_val;

  mrs_msgs::Llcp llcp_msg;

  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  llcp_publisher_.publish(llcp_msg);
  ROS_INFO_STREAM("[LlcpExample]: Sending servo message -> id: " << int(msg_out.id));

  res.message = "Servos set!";
  res.success = true;
  return true;
}

//}

/* //{ callbackSetTrigger() */

bool LlcpExample::callbackSetTrigger(mrs_msgs::SetTrigger::Request &req, mrs_msgs::SetTrigger::Response &res) {

  if (!is_initialized_) {
    res.success = false;
    return false;
  }

  trigger_msg msg_out;

  msg_out.id          = TRIGGER_MSG_ID;
  msg_out.trigger_num = req.trigger_num;
  msg_out.trigger     = req.trigger;


  mrs_msgs::Llcp llcp_msg;

  uint8_t *msg_ptr = (uint8_t *)&msg_out;

  for (int i = 0; i < sizeof(msg_out); i++) {
    llcp_msg.payload.push_back(msg_ptr[i]);
  }

  llcp_publisher_.publish(llcp_msg);
  ROS_INFO_STREAM("[LlcpExample]: Sending trigger message -> id: " << int(msg_out.id));

  res.message = "Trigger set!";
  res.success = true;
  return true;
}

//}

/* callbackReceiveMessage() //{ */

void LlcpExample::callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  uint8_t payload_size = msg->payload.size();
  uint8_t payload_array[payload_size];
  std::copy(msg->payload.begin(), msg->payload.end(), payload_array);

  switch (payload_array[0]) {
    case HEARTBEAT_MSG_ID: {
      heartbeat_msg *received_msg = (heartbeat_msg *)payload_array;
      received_msg->messages_received;
      ROS_INFO_STREAM("[LlcpExample]: Got HB msg, msgs: " << int(received_msg->messages_received) << " Servo1: " << int(received_msg->servo1_pos) << " Servo2: " << int(received_msg->servo2_pos) << " Last trig: " << int(received_msg->last_trigger) << " Last trig num: " << int(received_msg->last_trigger_num));
      break;
    }
    default: {
      ROS_ERROR_STREAM("[LlcpExample]: Received unknown message with id " << int(payload_array[0]));
      break;
    }
  }
}

//}

// | ------------------------ routines ------------------------ |


}  // namespace llcp_example

PLUGINLIB_EXPORT_CLASS(llcp_example::LlcpExample, nodelet::Nodelet);
