#include "enums.h"
#include "structs.h"
#include "mavlink/common/mavlink.h"
#include <stdint.h>
#include <HardwareSerial.h>
#include "conversion.h"

static const double pi = 3.14159265358979323846264338;
HardwareSerial *dune = &Serial;
struct sensordata S;
// MAVlink constants common to all messages
uint8_t sysid = 0, compid = 0;

struct wp_class current_pos, home;
struct Path path;
struct GPS_Fix gps_fix;
struct Compass cmps_read;
struct Target target;
struct XTRACK xtrack;

float heading, speed;
int op_mode;

void setup()
{
  Serial.begin(9600);
  op_mode = MANEUVER;
  home = {toRad(59.347019), toRad(18.074947), 0, 0};
  speed = 1;
  heading = 0;
  S.lat = home.lat;
  S.lon = home.lon;
  S.vs  = speed;
  gps_fix.lat = home.lat;
  gps_fix.lon = home.lon;
  gps_fix.status = 1;
  gps_fix.alt = 0;
  gps_fix.hdop = 0.666;
  gps_fix.sog = speed;
  gps_fix.nsats = 10;
}

void loop()
{
  // Do dead reckoning
  Navigate();
  heading = target.ctt;
  sendNavControllerOutput(target.dtt);

  // Send mavlink data
  GPS_mission_manager();
  sendHB();
  delay(250);
  sendRawGPS();
  sendAttitude();
  delay(250);
  sendGPS();

}
