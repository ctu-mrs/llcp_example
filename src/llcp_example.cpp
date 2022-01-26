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
  enum serial_receiver_state
  {
    WAITING_FOR_MESSSAGE,
    EXPECTING_SIZE,
    EXPECTING_PAYLOAD,
    EXPECTING_CHECKSUM
  };


  /* ros::Timer serial_timer_; */
  /* ros::Timer maintainer_timer_; */

  /* void interpretSerialData(uint8_t data); */
  /* void callbackSerialTimer(const ros::TimerEvent &event); */
  /* void callbackMaintainerTimer(const ros::TimerEvent &event); */

  /* bool callbackAll(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); */
  /* bool callbackLlcpExample(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); */
  /* bool callbackOuster(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); */

  void callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg);

  /* void callbackSendRawMessage(const mrs_msgs::SerialRawConstPtr &msg); */
  /* void callbackSendCommand(const mrs_msgs::LlcpExampleState &msg); */

  /* uint8_t connectToSensor(void); */
  /* void    processMessage(uint8_t payload_size, uint8_t *input_buffer, uint8_t checksum, uint8_t checksum_rec, bool checksum_correct); */

  ros::NodeHandle nh_;

  ros::Publisher  llcp_publisher_;
  ros::Subscriber llcp_subscriber_;

  /* ros::Publisher baca_protocol_publisher_; */

  /* ros::Subscriber gimbal_command_subscriber; */

  /* serial_port::SerialPort serial_port_; */

  /* boost::function<void(uint8_t)> serial_data_callback_function_; */

  /* bool publish_bad_checksum; */
  /* bool use_timeout; */

  /* uint16_t received_msg_ok           = 0; */
  /* uint16_t received_msg_ok_gimbal    = 0; */
  /* uint16_t received_msg_bad_checksum = 0; */

  /* int serial_rate_        = 5000; */
  /* int serial_buffer_size_ = 1024; */

  /* std::string portname_; */
  /* int         baudrate_; */
  /* std::string uav_name_; */

  /* std::mutex mutex_msg; */

  /* ros::Time interval_      = ros::Time::now(); */
  /* ros::Time last_received_ = ros::Time::now(); */

  /* bool is_connected_   = false; */
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
  /* maintainer_timer_ = nh_.createTimer(ros::Rate(1), &LlcpExample::callbackMaintainerTimer, this); */

  is_initialized_ = true;
}
//}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackSendRawMessage() //{ */

void LlcpExample::callbackReceiveMessage(const mrs_msgs::LlcpConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }
  ROS_INFO("[LlcpExample]: received llcp message");

  // llcp is working with arrays, so we need to convert the payload from the ROS message into an array
  std::vector<uint8_t> payload_vec  = msg->payload;
  uint8_t              payload_size = payload_vec.size();
  uint8_t              payload_arr[payload_size];
  std::copy(payload_vec.begin(), payload_vec.end(), payload_arr);

  example_msg *received_msg = (example_msg *)payload_arr;
  ROS_INFO_STREAM("[LlcpExample]: Received message:");
  ROS_INFO_STREAM("[LlcpExample]: data1: " << received_msg->data1);
  ROS_INFO_STREAM("[LlcpExample]: data2: " << received_msg->data2);
  ROS_INFO_STREAM("[LlcpExample]: data3: " << received_msg->data3);
}

//}

// | ------------------------ routines ------------------------ |


}  // namespace llcp_example

PLUGINLIB_EXPORT_CLASS(llcp_example::LlcpExample, nodelet::Nodelet);
