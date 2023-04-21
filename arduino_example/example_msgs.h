#define SERVO_MSG_ID 54
#define TRIGGER_MSG_ID 53
#define HEARTBEAT_MSG_ID 52

struct __attribute__((__packed__)) servo_msg
{
  uint8_t  id;
  uint16_t servo1_pos;
  uint16_t servo2_pos;
};

struct __attribute__((__packed__)) trigger_msg
{
  uint8_t  id;
  uint8_t  trigger_num;
  bool  trigger;
};

struct __attribute__((__packed__)) heartbeat_msg
{
  uint8_t  id;
  bool     is_running;
  uint16_t messages_received;
  uint16_t servo1_pos;
  uint16_t servo2_pos;
  bool     last_trigger;
  uint8_t  last_trigger_num;
};
