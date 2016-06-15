#ifndef __STRUCTS__
#define __STRUCTS__

#include <stdint.h>
#include <time.h>

struct wp_class {
  double lat; // [rad]
  double lon; // [rad]
  double alt; // [m]
  float depth;

  bool operator==(struct wp_class& other) {
    return (this->lat == other.lat) && (this->lon == other.lon);
  }
};

struct Vector3D {float x, y, z;};
struct Tilt {float roll, pitch;};

/*! This struct contains sensor data, i.e data which
    can originate from sensors or simulated sensors. */
struct sensordata {
  uint32_t  last_gps_fix_ms; //!<
  double    lat;            //!< [rad] Current latitude
  double    lon;            //!< [rad] Current longitude
  uint8_t   day; //!< [ms]   mseks since start of this week
  uint8_t   month;       //!< [weeks] since 1980
  uint8_t   year;
  int       gps_status;     //!< [-]      0,1,2,3
  int       time_utc;       //!< [HHMMSS]

  float     course2home;    //!< [rad];
  float     dist2home;      //!< [nm];

  float     depth;          //!< [m]
  float     rpm;            //!< [-]
  float     pitch;          //!< [rad]  CMPS10
  float     roll;           //!< [rad]  CMPS10
  float     cc;             //!< [rad]  CMPS10

  float     vs;             //!< [Knot]  Boat speed through water
  Vector3D  accels;         //!< [m/s2]
  Vector3D  gyros;          //!< [rad/s]

  float     yawrate;        //!< [rad/s]
  float     rudder;         //!< [rad]     In global reference system
  float     elevator;       //!< [rad]     In global reference system
  float     battery_voltage;   //!< [volt]  Power module voltage
  float     battery_current; //!< [A]     Power module current
  float     batt_Ah;        //!< [Ah]   Power module intigrated since start
  int       pressure_diff;  //!< [volt]    difference in pressure sensor readings
};

// GPS data
struct GPS_Fix {
  double    lat;     // [rad]
  double    lon;     // [rad]
  float     alt;     // [m]
  float     cog;     // [rad]
  double    sog;     // [m/s]
  uint8_t   nsats;   // [-] number of satellites
  uint8_t   status;  //uint8_t
  uint8_t   fix_quality;
  uint8_t   day, month, year;
  uint8_t   hour, minute, second;
  float     hdop;
  int       utc_time;
  char     *last_source; // "RMC", "GLL" or "GGA"
  float     StepCounter; // [m]
  unsigned long fix_time_ms;
  struct tm *time;
};

uint64_t time_epoch_usec(GPS_Fix &fix) { return (uint64_t)mktime(fix.time)*1000; }

struct Compass {
  float heading; // [Rad]
  float roll;    // [Rad]
  float pitch;   // [Rad]
  uint8_t status;   // 0 = Error, 1 = Heading OK
};

struct Path {
  double start_lat, start_lon, start_depth;   // Rads
  double end_lat, end_lon, end_depth;         // Rads
  double speed;                               // [m/s]
};

struct Target {
  float ctt;      // Course To Target
  float dtt;      // Distance To Target
  float depth;
  double err_cc;  // Heading error w.r.t. target bearing
};

// Cross track error control
enum {STATIC, DYNAMIC, OFF};
struct XTRACK {
  float k = 0.7;
  float S = 4;
  uint8_t type = STATIC;
};

#endif
