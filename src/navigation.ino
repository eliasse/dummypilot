const double M_TO_DEG    = 8.999280057595392e-06;    // Meters to Lat/Lon in degrees
const double M_TO_RAD    = 1.570670673141045e-07;    // Meters to Lat/Lon in radians

//! Call DeadReckoning() and calculate course2home and dist2home
static void Navigate()
{
  static unsigned long last_navigation = 0;
  if (millis() - last_navigation < 1000) {return;}

  last_navigation = millis();
  // Note that a valid GPS-fix automagically will be used, do not worry.
  DeadReckoning();
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

  gps_fix.lat = S.lat;
  gps_fix.lon = S.lon;
}

void GPS_mission_manager(){

  wp_class wpB        = {path.end_lat,  path.end_lon}; // End waypoint
  /* CCD   tmp           = distance_and_bearing(current_pos, wpB); */
  /* float ctt           = tmp.course; */
  /* float dtt           = tmp.dist;  // [NM] */

  current_pos.lat = S.lat;
  current_pos.lon = S.lon;

  wp_class wpA;

  // If the start wp in the path isn't set, set wpA to current position
  if ((abs(abs(path.start_lat) - toRad(1)) < toRad(1)) &&
      (abs(abs(path.start_lon) - toRad(1)) < toRad(1)))
    {
      wpA = current_pos;
    }
  else
    {
      wpA.lat = path.start_lat;
      wpA.lon = path.start_lon;
    }

  float dtt = calculateDistance(current_pos.lat, current_pos.lon, wpB.lat, wpB.lon);
  float ctt = calculateBearing(current_pos.lat, current_pos.lon, wpB.lat, wpB.lon);

  if ((dtt > 10) && (xtrack.type != OFF)) { ctt = course_wrt_xtrack(wpA, wpB, current_pos); }

  target.ctt     = ctt;  // [rad] Compass course
  target.dtt     = dtt;  // [rad] Compass course

  cmps_read.heading = ctt;
  S.cc = ctt;
}
