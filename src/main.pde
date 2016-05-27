#include "enums.h"
#include "structs.h"

// MAVlink constants common to all messages
uint8_t sysid = 0, compid = 0;
struct wp_class current_pos, home;
int op_mode;

void setup()
{
  Serial.begin(115200);
  op_mode = SERVICE;
  home = {59.347019, 18.074947, 0, 0};
}

void loop()
{
  switch (op_mode)
  {
    case SERVICE:
    break;
    case MANEUVER:
      // Do dead reckoning
    break;
  }
}
