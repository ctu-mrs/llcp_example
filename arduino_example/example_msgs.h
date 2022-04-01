#define DATA_MSG_ID 52
#define HEARTBEAT_MSG_ID 51

struct __attribute__((__packed__)) data_msg
{
  uint8_t  id;
  uint8_t  data1_uint8;
  uint32_t data2_uint32;
  float    data3_float;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  uint8_t  id;
  bool     is_running;
  uint16_t messages_received;
};
