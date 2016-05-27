const double M_TO_DEG    = 8.999280057595392e-06;    // Meters to Lat/Lon in degrees
const double M_TO_RAD    = 1.570670673141045e-07;    // Meters to Lat/Lon in radians

//! Call DeadReckoning() and calculate course2home and dist2home
static void Navigate()
{
  navigation_ms = time_ms;

  // Note that a valid GPS-fix automagically will be used, do not worry.
  DeadReckoning();

  S.course2home = calculateBearing(S.lat, S.lon, homepos.lat, homepos.lon);
  S.dist2home   = calculateDistance(S.lat, S.lon, homepos.lat, homepos.lon);
}

//! Increment position based on speed and heading
static void DeadReckoning()
{
  static unsigned long last_dead_reckoning_ms = 0;

  // Integrate lat/lon based on dead reckoning
  float dt = (float)( millis() - last_dead_reckoning_ms )/1000.0; // [sec]

  last_dead_reckoning_ms = millis();

  float v         = S.vs * cos(S.cc);    // [m/s] boat speed  in north direction
  float u         = S.vs * sin(S.cc);    // [m/s] boat speed  in east-direction

  double mn       = dt*v;    // [m] Distance travelled in north-direction
  double me       = dt*u;    // [m] Distance travelled in east-direction

  double dlat     = (mn * M_TO_RAD);
  double dlon     = (me * M_TO_RAD) / cos(S.lat);

  S.lat += dlat;	                 // [rad] craft position
  S.lon += dlon;	                 // [rad] craft position
}
