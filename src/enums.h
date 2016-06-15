#ifndef __ENUMS__
#define __ENUMS__
/* Vehicle status defined to resemble Operation Mode
   field of the IMC vehicle state message. */
enum OPERATION_MODE {
  SERVICE = 0,            // Ready to service requests
  CALIBRATION = 1,        // Ongoing vehicle calibration
  ERROR = 2,              // Vehicle is in error
  MANEUVER = 3,           // Maneuver active
  EXTERNAL_CONTROL = 4,   // External control (RC)
  BOOT = 5                // System is booting
};

#endif
