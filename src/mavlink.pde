void sendHB(){
  static uint32_t timer = 0;

  if ((millis() - timer > 5000) || (timer == 0)) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t system_mode = MAV_MODE_AUTO_ARMED; // Ignored
    uint32_t custom_mode = 11; // Used!
    uint8_t system_state = MAV_STATE_STANDBY;  // Ignored
    uint8_t system_type = MAV_TYPE_GROUND_ROVER; // Used!
    uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; // Ignored

    // Pack message
    mavlink_msg_heartbeat_pack(sysid, compid, &msg,
			       system_type, autopilot_type, system_mode,
			       custom_mode, system_state);
    // Copy message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
    // Send message
    dune->write(buf,len);
    timer = millis();
  }
}

void sendRawGPS(void)
{
  static uint32_t timer = 0;

  if (millis() - timer > 500) {

    mavlink_message_t msg;

    mavlink_msg_gps_raw_int_pack(0,0,&msg,
				 (uint64_t)micros(),
				 (uint8_t)gps_fix.status,
				 (int32_t)toDeg(gps_fix.lat)*1E7,
				 (int32_t)toDeg(gps_fix.lon)*1E7,
				 (int32_t)(gps_fix.alt*10UL),
				 (uint16_t)gps_fix.hdop*100,
				 (uint16_t)65535,
				 (uint16_t)gps_fix.sog*100,
				 (uint16_t)toDeg(heading)*100,
				 (uint8_t)gps_fix.nsats);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
    dune->write(buf,len);
    timer = millis();
  }
}

void sendAttitude()
{
  static float hdg;

  if (cmps_read.heading != hdg) {

    hdg = cmps_read.heading;

    mavlink_message_t msg;

    mavlink_msg_attitude_pack(sysid, compid, &msg,
			      (uint32_t)millis(),
			      cmps_read.roll,
			      cmps_read.pitch,
			      unwrap_pi(hdg),
			      0,
			      0,
			      0);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    dune->write(buf,len);
  }
}

void sendGPS()
{
  static uint32_t timer = 0;

  if (millis() - timer > 750)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      mavlink_msg_global_position_int_pack(sysid, compid, &msg,
					   millis(),
					   toDeg(gps_fix.lat)*1E7,   // in 1E7 degrees
					   toDeg(gps_fix.lon)*1E7,   // in 1E7 degrees
					   gps_fix.alt*10UL,         // millimeters above sea level
					   gps_fix.alt*10UL,         // millimeters above ground
					   gps_fix.sog*cos(cmps_read.heading)*100,  // X speed cm/s (+ve North)
					   gps_fix.sog*sin(cmps_read.heading)*100,  // Y speed cm/s (+ve East)
					   0,  // Z speed cm/s (+ve up)
					   toDeg(cmps_read.heading)*100);

      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      dune->write(buf,len);
      timer = millis();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Message help functions
void sendNavControllerOutput(float dtt)
{
  mavlink_message_t msg;

  float nav_roll = 0;           // Desired Roll
  float nav_pitch = 0;          // Desired Pitch
  int16_t nav_bearing = toDeg(target.ctt);    // Desired Heading
  int16_t target_bearing = toDeg(target.ctt); // Target bearing
  uint16_t wp_dist = 100;
  float alt_error = 0;
  float aspd_error = 0;         // Airspeed error [m/s]
  int xtrack_error = 0;

  mavlink_msg_nav_controller_output_pack(0,0,&msg,nav_roll, nav_pitch, nav_bearing,
                                         target_bearing, wp_dist, alt_error, aspd_error,
                                         xtrack_error);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  dune->write(buf,len);
}

// void sendMissionCurrent()
// {
//   mavlink_message_t msg;
//
//   mavlink_msg_mission_current_pack(0,0,&msg,0);
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
//   dune->write(buf,len);
// }

void sendSystemTime()
{
  mavlink_message_t msg;

  mavlink_msg_system_time_pack(0,0,&msg,time_epoch_usec(gps_fix),millis());

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  dune->write(buf,len);
}

void sendNamedValueFloat(char name[10], float value)
{
  mavlink_message_t msg;
  uint32_t time_boot_ms = 0;

  mavlink_msg_named_value_float_pack(0,0,&msg,time_boot_ms,name,value);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  dune->write(buf,len);
}

void sendNamedValueInt(char name[10], int32_t value)
{
  mavlink_message_t msg;
  uint32_t time_boot_ms = 0;

  mavlink_msg_named_value_int_pack(0,0,&msg,time_boot_ms,name,value);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  dune->write(buf,len);
}
void sendDebugVect(char name[10], float x, float y, float z)
{
  /* Debug message that can send a name and some floats (i.e. lat, lon, alt). */
  mavlink_message_t msg;
  uint32_t time_usec = 0; // Timestamp

  mavlink_msg_debug_vect_pack(0,0,&msg,name,time_usec,x,y,z);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
  dune->write(buf,len);
}

void sendMissionItem(uint16_t seq)
{
  /* This method won't affect the behavior of the Ardupilot task unless
  a MAV_CMD_TAKEOFF or MAV_CMD_LAND is contained in it. The task uses this to determine
  wether the vehicle is in a critical state or not. */

  mavlink_message_t msg;

  mavlink_msg_mission_item_pack(0,0,&msg,0,0,seq,MAV_FRAME_GLOBAL,MAV_CMD_NAV_WAYPOINT,0,1,0,0,0,0,
                                path.end_lat,path.end_lon,-path.end_depth);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);

  dune->write(buf,len);
}

void sendRawSensors()
{
  mavlink_message_t msg;
  // Hämta sensordata
  // Skicka meddelande
}

// void sendSystemStatus() // Battery levels and power consumption. CURRENTLY DUMMY LEVEL!
// {
//   static uint32_t time_since_last_send = 0;
//
//   if (time_ms - time_since_last_send <= 5000){ return; }
//   else {time_since_last_send = time_ms;}
//
//   battery_mon.read();
//
//   uint32_t o_s_present = 0; // Bitmask describing Onboard Sensors, not used by Task.
//   uint32_t o_s_enabled = 0; // Not checked by Task.
//   uint32_t o_s_health = 0; // Not checked by Task.
//   uint16_t load = 0; // Not checked by Task.
//
//   uint16_t voltage = battery_mon.voltage()*1000; // [mV]
//   int16_t  current = battery_mon.current_amps()*1000; // [mA]
//   //battery_mon.current_total_mah();
//   int8_t battery_remaining = 100; // Dummy value that says 100% battery left
//
//   uint16_t drop_rate_comm = 0;
//   uint16_t err_comm = 0;
//   uint16_t err_count1 = 0;
//   uint16_t err_count2 = 0;
//   uint16_t err_count3 = 0;
//   uint16_t err_count4 = 0;
//
//   mavlink_message_t msg;
//   mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, o_s_present,
//                               o_s_enabled, o_s_health, load, voltage, current, battery_remaining,
//                               drop_rate_comm, err_comm, err_count1, err_count2, err_count3,
//                               err_count4);
//
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   // Copy message to send buffer
//   uint16_t len = mavlink_msg_to_send_buffer(buf,&msg);
//   // Send message
//   dune->write(buf,len);
// }

// ====================================== STATUSTEXT =============
void sendStatusText(uint8_t severity, const char& text)
{
  mavlink_message_t msg;

  mavlink_msg_statustext_pack(sysid, compid, &msg,
                              severity,
                              &text);
  uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  dune->write(buf, len);
}
