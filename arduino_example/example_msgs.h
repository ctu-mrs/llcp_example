#define DATA_MSG_ID 52
#define HEARTBEAT_MSG_ID 51

struct __attribute__((__packed__)) data_msg
{
  const uint8_t id = DATA_MSG_ID;
  uint8_t       data1_uint8;
  uint32_t      data2_uint32;
  float         data3_float;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  const uint8_t id = HEARTBEAT_MSG_ID;
  bool          is_running;
  uint16_t      messages_received;
};
