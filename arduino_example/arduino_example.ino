extern "C" {
#include <llcp.h>
}

#include "example_msgs.h"


#define TX_BUFFER_LEN 255
uint8_t tx_buffer[TX_BUFFER_LEN];

LLCP_Receiver_t llcp_receiver;

uint8_t             my_data1_uint8 = 42;
uint32_t             my_data2_uint32 = 420;
float               my_data3_float = 420.69;
uint16_t num_msg_received = 0;

long last_hb = millis();

void setup() {

  Serial.begin(115200);

  // inicialize the llcp receiver struct
  llcp_initialize(&llcp_receiver);

}

void loop() {

  if (receive_message()) {
    send_data();
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

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}

void send_data() {

  data_msg my_msg;
  uint16_t msg_len;

  // fill the message with data
  my_msg.id = DATA_MSG_ID;
  my_msg.data1_uint8 = my_data1_uint8;
  my_msg.data2_uint32 = my_data2_uint32;
  my_msg.data3_float = my_data3_float;

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
          case DATA_MSG_ID: {

              data_msg *received_msg = (data_msg *)llcp_message_ptr;

              my_data1_uint8 = received_msg->data1_uint8;
              my_data2_uint32 = received_msg->data2_uint32;
              my_data3_float = received_msg->data3_float;

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
