extern "C" {
#include <llcp.h>
}

#include "example_msgs.h"

#define TX_BUFFER_LEN 255
uint8_t tx_buffer[TX_BUFFER_LEN];

LLCP_Receiver_t llcp_receiver;

uint16_t num_msg_received = 0;

long last_hb = millis();

bool trigger_cmd = false;
uint8_t trigger_num = 0;
bool got_new_trigger = false;

uint16_t servo1_pos_cmd = 1000;
uint16_t servo2_pos_cmd = 1000;
bool got_new_servo_pos = false;

void setup() {

  Serial.begin(115200);
  // inicialize the llcp receiver struct
  llcp_initialize(&llcp_receiver);

}

void loop() {

  if (receive_message()) {

    if (got_new_servo_pos) {
      got_new_servo_pos = false;
      //TODO -  do stuff with new servo command here, variables servo1_pos_cmd and servo2_pos_cmd have new values!
    }

    if (got_new_trigger) {
      got_new_trigger = false;
      //TODO -  do stuff with new trigger command here, trigger_cmd and trigger_num have new values!
    }

  }

  if (millis() - last_hb >= 1000) {
    last_hb = millis();
    send_heartbeat();
  }

  delay(10);
}

void send_heartbeat() {

  heartbeat_msg my_msg;
  uint16_t msg_len;

  // fill the message with data

  my_msg.id = HEARTBEAT_MSG_ID;
  my_msg.is_running = true;
  my_msg.messages_received = num_msg_received;
  my_msg.servo1_pos = servo1_pos_cmd;
  my_msg.servo2_pos = servo2_pos_cmd;
  my_msg.last_trigger = trigger_cmd;
  my_msg.last_trigger_num = trigger_num;  

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}

bool receive_message() {
  uint16_t msg_len;
  bool got_valid_msg = false;
  LLCP_Message_t* llcp_message_ptr;

  while (Serial.available() > 0) {
    bool checksum_matched;
    uint8_t char_in = Serial.read();

    //individual chars are processed one by one by llcp, if a complete message is received, llcp_processChar() returns true
    if (llcp_processChar(char_in, &llcp_receiver, &llcp_message_ptr, &checksum_matched)) {
      if (checksum_matched) {
        num_msg_received++;
        switch (llcp_message_ptr->payload[0]) {
          case SERVO_MSG_ID: {

              servo_msg *received_msg = (servo_msg *)llcp_message_ptr;

              servo1_pos_cmd = received_msg->servo1_pos;
              servo2_pos_cmd = received_msg->servo2_pos;

              got_new_servo_pos = true;
              got_valid_msg = true;
              break;
            }
          case TRIGGER_MSG_ID: {

              trigger_msg *received_msg = (trigger_msg *)llcp_message_ptr;

              trigger_cmd = received_msg->trigger;
              trigger_num = received_msg->trigger_num;

              got_new_trigger = true;
              got_valid_msg = true;
              break;
            }
          case HEARTBEAT_MSG_ID: {
              got_valid_msg = true;
              break;
            }
        }

        return true;
      }
    }
  }
  return got_valid_msg;
}
