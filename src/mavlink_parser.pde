void readMessage()
{
	mavlink_message_t msg;
	mavlink_status_t status;
	char* text;

	while(dune->available()) {
		uint8_t c = dune->read();

		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                                     //handleRequestDataStream(&msg);
			             break;

                                case MAVLINK_MSG_ID_HEARTBEAT:
                                     text = "APM Heartbeat message received";
                                     sendStatusText(MAV_SEVERITY_DEBUG,*text);
                                     break;
                                case MAVLINK_MSG_ID_MISSION_COUNT:
                                     // message telling how many WPs to be sent
                                     //handleMissionCount(&msg);
                                     break;
                                case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
                                     text = "WRITE PARTIAL LIST RECEIVED";
                                     sendStatusText(MAV_SEVERITY_DEBUG,*text);
                                     break;
                                case MAVLINK_MSG_ID_MISSION_ITEM: // something like WP
                                     handleMissionItem(&msg);
                                     break;
                                case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                                     //handleMissionSetCurrent(&msg);
                                     break;
                                case MAVLINK_MSG_ID_MISSION_REQUEST:
                                     //handleMissionRequest(&msg);
                                     break;
                                case MAVLINK_MSG_ID_SET_MODE: // Ignore this message
                                     // FBWB för helikopter eller GUIDED med DUNE-kontroller
                                     //handleSetMode(&msg);
                                     text = "Set MODE received, ignoring!";
                                     sendStatusText(MAV_SEVERITY_DEBUG,*text);
                                     break;
                                case MAVLINK_MSG_ID_NAMED_VALUE_INT: // Used to receive heading from DUNE
                                     handleNamedValueInt(&msg);
                                     break;
				default:
				      // Busy doing nothing at all, don't bother to call
				break;
			}
		}
	}
}

void handleNamedValueInt(mavlink_message_t* msg)
{
  mavlink_named_value_int_t m_int;
  mavlink_msg_named_value_int_decode(msg,&m_int);

  //if (hdg == FromDUNE){ cc = ToRad(m_int.value/10); }

  // Verifying that the right value was received
  //if (DEBUG_HEADING) {char * n = "Headbounce"; sendNamedValueFloat(n,heading);}
  //if (m_int.name == Heading... kolla upp strcmp
}

/*
void handleWritePartialList(mavlink_message_t* msg)
{
  mavlink_mission_write_partial_list_t m_part_list;
  mavlink_msg_mission_write_partial_list_decode(msg,&m_part_list);

  if (m_part_list.start_index <= num_mission_items)
  {
    char* text = "PARTIAL LIST START_INDEX MISMATCH";
    sendStatusText(MAV_SEVERITY_DEBUG,*text);
  }
  else if (m_part_list.start_index == num_mission_items + 1)
  {
    // Set number of mission items to include the new wps.
    num_mission_items = m_part_list.end_index; // int16_t
    // Request waypoint
    sendMissionRequest(m_part_list.start_index);
  }
}

void handleMissionRequest(mavlink_message_t* msg)
{
  mavlink_mission_request_t m_request;
  mavlink_msg_mission_request_decode(msg,&m_request);

  // Create and send a message with information on the requested item
  sendMissionItem(m_request.seq);
}

void handleMissionSetCurrent(mavlink_message_t* msg)
{
  mavlink_mission_set_current_t m_set_current;
  mavlink_msg_mission_set_current_decode(msg,&m_set_current);
  current_wp = m_set_current.seq;
}
*/
void handleMissionItem(mavlink_message_t* msg)
{
  mavlink_mission_item_t mitem;
  mavlink_msg_mission_item_decode(msg,&mitem);

  char *vect_name = "WPLOC";
  // Check what the message contains
  switch (mitem.command)
  {
    default:
      break;
    case (MAV_CMD_NAV_WAYPOINT): // Skickas i grader men vill göra om till samma som Mission Manager och nav_math vill ha
      path.start_lat = 0;
      path.start_lon = 0;
      path.start_depth = 0;
      path.end_lat = toRad(mitem.x);
      path.end_lon = toRad(mitem.y);
      path.end_depth = mitem.z; //altitude (relative or absolute, depending on frame)

      sendDebugVect(vect_name, path.end_lat,
                               path.end_lon,
                               path.end_depth); // For debugging


      sendHB(); // The MODE parameter is passed to DUNE throught the HeartBeat message
      break;
    case (MAV_CMD_NAV_LOITER_UNLIM):
      // Ignore for now
      break;
    case (MAV_CMD_NAV_TAKEOFF):
      // Ignore...
      break;
    case (MAV_CMD_DO_CHANGE_SPEED):
      char *text = "DO CHANGE SPEED RECEIVED";
      sendStatusText(MAV_SEVERITY_DEBUG,*text);
      break;

  }

	GPS_mission_manager();
  /*
  // Request the next waypoint if not all has been received
  if (receive_wp < num_mission_items)
  {
    sendMissionRequest(receive_wp+1);
  }
  else if (mitem.seq == num_mission_items)
  {
    // send mission accepted message
    sendMissionAck();
  }
  */
}
/*
// Mission accepted message (sent when all WPs received)
void sendMissionAck()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_ack_pack(0,0,&msg,
            mavlink_system.sysid,mavlink_system.compid,MAV_MISSION_ACCEPTED);
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  hal.uartA->write(buf,len);

  current_wp = 1;
  MODE=AUTOMATIC;

  char* text = "MISSION ACK SENT, AUTOMATIC MODE";
  sendStatusText(MAV_SEVERITY_DEBUG,*text);
}

void handleMissionCount(mavlink_message_t* msg)
{
  mavlink_mission_count_t mcount;
  mavlink_msg_mission_count_decode(msg,&mcount);
  // The message contains the number of waypoints that DUNE wants to send.
  // Let's reset the current waypoint!
  current_wp = 0;
  // Let's tell DUNE it can start sending the first waypoint!
  // mcount.count is the number of waypoints to receive.
  num_mission_items = mcount.count;

  char* text = "MISSION COUNT RECEIVED";
  sendStatusText(MAV_SEVERITY_DEBUG,*text);

  sendMissionRequest(0);
}

void sendMissionRequest(uint16_t seq) // seq is the mission item number that we want
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_request_pack(mavlink_system.sysid,mavlink_system.compid,&msg,0,0,seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  hal.uartA->write(buf,len);
}

void handleRequestDataStream(mavlink_message_t* msg)
{
  mavlink_request_data_stream_t dstr;
  mavlink_msg_request_data_stream_decode(msg, &dstr);
  uint8_t sid = dstr.req_stream_id;

  char* text;

  switch (sid)
  {
    case MAV_DATA_STREAM_EXTRA1:
      text = "Request for EXTRA1 received";
      sendStatusText(MAV_SEVERITY_DEBUG,*text);
    default:
      text = "Data Stream Request received, unknown STREAM ID";
      sendStatusText(MAV_SEVERITY_DEBUG,*text);
  }
}
*/
