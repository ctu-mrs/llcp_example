
#include "llcp.h"
#include "example_msgs.h"


#define TX_BUFFER_LEN 255
uint8_t tx_buffer[TX_BUFFER_LEN];

LLCP_Receiver_t llcp_receiver;
LLCP_Message_t llcp_message_in;


void setup() {

  Serial.begin(115200);

  // inicialize the llcp receiver struct
  llcp_initialize(&llcp_receiver);

}

void loop() {

  if (receive_message(&llcp_message_in)) {
    send_message();
  }

  send_message();
  delay(1000);

}

void send_message() {

  example_data_msg my_msg;
  uint16_t msg_len;

  // fill the message with data
   // my_msg.id = 52;
  my_msg.data1 = 121;
  my_msg.data2 = 5609;
  my_msg.data3 = 128999;

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer, my_msg.id);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}

bool receive_message(LLCP_Message_t* llcp_message_in) {
  uint16_t msg_len;

  while (Serial.available() > 0) {
    bool checksum_matched;
    uint8_t char_in = Serial.read();

    if (llcp_processChar(char_in, &llcp_receiver, &llcp_message_in, &checksum_matched)) {
/*
      switch (llcp_message_in->id) {
        case example_data_msg::id: {
            Serial.println("pes");
            break;
          }
        case example_heartbeat_msg::id: {
            Serial.println("kocka");
            break;
          }
      }*/





      return true;
    }
  }
  return false;
}
