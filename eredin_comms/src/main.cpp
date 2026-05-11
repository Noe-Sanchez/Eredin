//#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "secrets.h"
#include "common/mavlink.h"

union px4_custom_mode {
  struct {
    uint16_t reserved;
    uint8_t main_mode;
    uint8_t sub_mode;
  };
  uint32_t data;
  float data_float;
  struct {
    uint16_t reserved_hl;
    uint16_t custom_mode_hl;
  };
};

// RTOS handles
void Task_ErrorHandler  ( void *pvParameters );
void Task_WifiHandler   ( void *pvParameters );
void Task_SendHeartbeat ( void *pvParameters );
void Task_VCCommsHandler( void *pvParameters );

QueueHandle_t queue_outbound;

// Mock variable for mavlink testing with vc
float mav_heading;
HardwareSerial VCSerial(1);

void setup() {
  //Serial.begin(115200, SERIAL_8N1, 16, 17); // Initialize serial communication
  // Serial is usb
  // VCSerial is comms with vc, on pins 16 and 17
  VCSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("Starting RTOS");

  // This is memory intensive, remove in the future
  queue_outbound = xQueueCreate(10, sizeof(mavlink_message_t));

  xTaskCreate(Task_WifiHandler,    "WifiHandler",    10000, NULL, 1, NULL);
  xTaskCreate(Task_SendHeartbeat,  "SendHearbeat",   10000, NULL, 1, NULL);
  xTaskCreate(Task_VCCommsHandler, "VCCommsHandler", 10000, NULL, 1, NULL);
  
  vTaskDelete(NULL);
  
}

void Task_VCCommsHandler( void *pvParameters ) {
  // Parse from vc and send via mavlink
  mavlink_message_t msg_out;
  uint8_t out_buf[MAVLINK_MAX_PACKET_LEN];
  
  while(1) {
    /*
    //mavlink_msg_vfr_hud_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, 0, 0, mav_heading, 0, 0, 0);
    //if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send VFR HUD message to queue!"); }
    mavlink_msg_attitude_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, 0, 0, 0, mav_heading*3.14/180, 0, 0, 0);
    if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send attitude message to queue!"); }
    mav_heading += 10.0;
    if (mav_heading >= 360.0) { mav_heading = 0.0; }
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    */

    // Read from VCSerial and parse for heading updates
    if (VCSerial.available()) {
      String line = VCSerial.readStringUntil('\n');
      if (line.startsWith("HEADING:")) {
	String heading_str = line.substring(8);
        float heading = heading_str.toFloat();
	mav_heading = heading;
      }
    }
    mavlink_msg_attitude_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, 0, 0, 0, mav_heading*3.14/180, 0, 0, 0);
    if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send attitude message to queue!"); }

    vTaskDelay(200 / portTICK_PERIOD_MS);

  }
    
}

void Task_WifiHandler( void *pvParameters ) {
  // Only handler owns this
  //WiFiClient client;
  WiFiUDP udp;
  mavlink_message_t msg_out;
  uint8_t  out_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t out_len;

  mavlink_message_t msg_in;
  uint8_t  in_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t in_len;

  udp.begin(14550);

  bool handled_command = false;
  while(1) {
    // Process sending first
    if(xQueueReceive(queue_outbound, &msg_out, 1) == pdPASS) {
      udp.beginPacket("10.42.0.1", 14550);
      out_len = mavlink_msg_to_send_buffer(out_buf, &msg_out);
      udp.write(out_buf, out_len);
      udp.endPacket();
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
      // Constrain read size to prevent buffer overflow
      int readSize = (packetSize > MAVLINK_MAX_PACKET_LEN) ? MAVLINK_MAX_PACKET_LEN : packetSize;
      
      // Read the data into the buffer
      int in_len = udp.read(in_buf, readSize);
      
      if (in_len > 0) {
        // Store UDP sender info as requested
        IPAddress senderIP  = udp.remoteIP();
        uint16_t senderPort = udp.remotePort();

	// Parse mavlink
	for (int i = 0; i < in_len; i++) {
	  if (mavlink_parse_char(MAVLINK_COMM_0, in_buf[i], &msg_in, NULL)) {

            if (msg_in.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
	      handled_command = false;
	      
	      mavlink_command_long_t cmd;
	      mavlink_msg_command_long_decode(&msg_in, &cmd);
	      Serial.print("Received command ");
	      Serial.println(cmd.command);

              if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
		if (cmd.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {

		  const uint8_t * version_bytes = (const uint8_t *) "5b85859";
	          const uint8_t * uid2 = (const uint8_t *) "DanS";

		  mavlink_msg_autopilot_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, (uint64_t)8223, 2, 2, 2, 52, version_bytes, version_bytes, version_bytes, 45, 1, 1, uid2); 
	          if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send autopilot version message to queue!"); }
		  handled_command = true;
		} else if (cmd.param1 == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
		  mavlink_msg_protocol_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, 2, 2, 2, (const uint8_t *) "DanS", (const uint8_t *) "DanS");

	          if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send protocol version message to queue!"); }
		  handled_command = true;
		}
	      } // End request message handling
	      // For now, we only handle request message cmd

	      if (handled_command) {
	        // Send command acknowledgment
	        mavlink_msg_command_ack_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, cmd.command, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
	        if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send command acknowledgment message to queue!"); }
	      } else {
	        // Send command acknowledgment for unhandled command
	        mavlink_msg_command_ack_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, cmd.command, MAV_RESULT_UNSUPPORTED, 100, 0, 1, 1);
	        if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send command acknowledgment message to queue!"); }
	      }// End of command queueing
	    
	    } else if (msg_in.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
	      int param_index = 0;
	      int param_count = 5;
	      mavlink_param_union_t param_value;

	      mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, "test", 0, MAV_PARAM_TYPE_INT32, param_index, param_count);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send param value message to queue!"); }
	      param_index++;

	      param_value.param_int32 = 4001;
	      mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, "SYS_AUTOSTART", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send param value message to queue!"); }
              param_index++;

	      param_value.param_int32 = 2;
	      mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, "MAV_SYS_ID", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send param value message to queue!"); }
	      param_index++;

	      param_value.param_int32 = 0;
	      mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, "SYS_AUTOCONFIG", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send param value message to queue!"); }
	      param_index++;
	      
	      param_value.param_int32 = 0;
	      mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, "COM_RC_IN_MODE", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send param value message to queue!"); }
              	    
	    } else if (msg_in.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
	      mavlink_msg_mission_count_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg_out, 1, MAV_COMP_ID_AUTOPILOT1, 0, MAV_MISSION_TYPE_MISSION, 1);
	      if (xQueueSend(queue_outbound, &msg_out, 1) != pdPASS) { Serial.println("Failed to send mission count message to queue!"); }
	    }
	  }
	}
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

  }
}

void Task_ErrorHandler( void *pvParameters ) {
  while(1) {
    Serial.println("Error occurred!");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void Task_SendHeartbeat(void *pvParameters) {
  union px4_custom_mode custom_mode;
  mavlink_message_t heart_msg;
  uint8_t  heart_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t heart_len;

  while(1) {
    custom_mode.data = 0;
    custom_mode.main_mode = 1;
    custom_mode.sub_mode = 0;
    mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &heart_msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 13, custom_mode.data, MAV_STATE_STANDBY); 
    
    if (xQueueSend(queue_outbound, &heart_msg, 1) != pdPASS) {
      Serial.println("Failed to send heartbeat message to queue!");
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 1 second

  }
}


void loop() {
  // Leave empty
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}
