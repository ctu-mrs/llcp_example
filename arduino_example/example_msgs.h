struct __attribute__((__packed__)) data_msg
{
  static const uint8_t id = 52;
  uint8_t              data1_uint8;
  uint32_t             data2_uint32;
  float                data3_float;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  static const uint8_t id = 51;
  bool                 is_running;
  uint16_t             messages_received;
};
