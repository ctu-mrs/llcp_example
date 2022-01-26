struct __attribute__((__packed__)) example_data_msg
{
  static const uint8_t id = 52;
  uint8_t       data1;
  uint16_t      data2;
  uint32_t      data3;
};

struct __attribute__((__packed__)) example_heartbeat_msg
{
  static const uint8_t id = 51;
  uint32_t      heartbeat;
};
