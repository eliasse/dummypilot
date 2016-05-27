// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// To/From ECEF transformations are borrowed frome LSTS/DUNE.
//! Semi-major axis.
static const double c_wgs84_a = 6378137.0;
//! Semi-minor axis.
static const double c_wgs84_b = 6356752.3142;
//! First eccentricity squared.
static const double c_wgs84_e2 = 0.00669437999013;
//! Second (prime) eccentricity squared.
static const double c_wgs84_ep2 = 0.00673949674228;
//! Flattening.
static const double c_wgs84_f = 0.0033528106647475;

//! Compute the radius of curvature in the prime vertical (Rn).
//!
//! @param[in] lat WGS-84 latitude (rad).
//!
//! @return radius of curvature in the prime vertical (rad).
template <typename Type>
static inline Type
computeRn(Type lat)
{
  double lat_sin = sin(lat);
  return c_wgs84_a / sqrt(1 - c_wgs84_e2 * (lat_sin * lat_sin));
}

//! Convert WGS-84 coordinates to ECEF (Earch Center Earth Fixed) coordinates.
//!
//! @param[in] lat WGS-84 latitude (rad).
//! @param[in] lon WGS-84 longitude (rad).
//! @param[in] hae WGS-84 coordinate height (m).
//! @param[out] x storage for ECEF x coordinate (m).
//! @param[out] y storage for ECEF y coordinate (m).
//! @param[out] z storage for ECEF z coordinate (m).
//template<typename Ta, typename Tb, typename Tc>
static void toECEF(double lat, double lon, double hae, double* x, double* y, double* z)
{
  double cos_lat = cos(lat);
  double sin_lat = sin(lat);
  double cos_lon = cos(lon);
  double sin_lon = sin(lon);
  double rn = computeRn(lat);

  *x = (rn + hae) * cos_lat * cos_lon;
  *y = (rn + hae) * cos_lat * sin_lon;
  *z = (((1.0 - c_wgs84_e2) * rn) + hae) * sin_lat;
}

//! Convert ECEF (x,y,z) to WGS-84 (lat, lon, hae).
//!
//! @param[in] x ECEF x coordinate (m).
//! @param[in] y ECEF y coordinate (m).
//! @param[in] z ECEF z coordinate (m).
//! @param[out] lat WGS-84 latitude (rad).
//! @param[out] lon WGS-84 longitude (rad).
//! @param[out] hae height above WGS-84 ellipsoid (m).
//template <typename Ta, typename Tb, typename Tc, typename Td>
static void fromECEF(double x, double y, double z, double* lat, double* lon, double* hae)
{
  double p = sqrt(x * x + y * y);
  *lon = atan2(y, x);
  *lat = atan2(z / p, 0.01);
  double n = computeRn(*lat);
  *hae = p / cos(*lat) - n;
  double old_hae = -1e-9;
  double num = z / p;

  while (fabs(*hae - old_hae) > 1e-4)
    {
      old_hae = *hae;
      double den = 1 - c_wgs84_e2 * n / (n + *hae);
      *lat = atan2(num, den);
      n = computeRn(*lat);
      *hae = p / cos(*lat) - n;
    }
}

// Closest point on line (ECEF coordinates)
// x_10: x1, x_11: y_1, x_12: z_1 (WP A)
// x_20: x2, x_21: y_2, x_22: z_2 (WP B)
// x_30: x2, x_31: y_3, x_32: z_3 (Vehicle position)
void closestpoint(double x_10, double x_11, double x_12,
		  double x_20, double x_21, double x_22,
		  double x_00, double x_01, double x_02,
		  double *rx, double *ry, double *rz)
{
  double t = 0;

  // x_1 - x_0
  double X10[3] = {x_10 - x_00, x_11 - x_01, x_12 - x_02};
  double X21[3] = {x_20 - x_10, x_21 - x_11, x_22 - x_12};
  double dprod = X10[0]*X21[0] + X10[1]*X21[1] + X10[2]*X21[2];
  double denom = pow(X21[0],2) + pow(X21[1],2) + pow(X21[2],2);
  t -= dprod/denom;

  *rx = x_10 + (x_20 - x_10)*t;
  *ry = x_11 + (x_21 - x_11)*t;
  *rz = x_12 + (x_22 - x_12)*t;
}

float course_wrt_xtrack(struct wp_class wpa, struct wp_class wpb, struct wp_class pos)
{
    float k_tmp = xtrack.k;

    double ax, ay, az, bx, by, bz, px, py, pz;
    toECEF(wpa.lat, wpa.lon, 0, &ax, &ay, &az);
    toECEF(wpb.lat, wpb.lon, 0, &bx, &by, &bz);
    toECEF(pos.lat, pos.lon, 0, &px, &py, &pz);

    double cpx, cpy, cpz; // Closest point
    closestpoint(ax, ay, az,
                 bx, by, bz,
                 px, py, pz,
                 &cpx, &cpy, &cpz);

    if (xtrack.type == DYNAMIC)
        {
           wp_class wpC;
           fromECEF(cpx, cpy, cpz, &wpC.lat, &wpC.lon, &wpC.alt);
           float XTE    = calculateDistance(pos.lat, pos.lon, wpC.lat, wpC.lon);
           float L      = calculateDistance(wpa.lat, wpa.lon, wpb.lat, wpb.lon);
           xtrack.k     = (XTE > xtrack.S) ? 1 : 1 - xtrack.S/L;
        }

    double x, y, z;
    x = cpx + (bx - cpx) * (1 - xtrack.k);
    y = cpy + (by - cpy) * (1 - xtrack.k);
    z = cpz + (bz - cpz) * (1 - xtrack.k);

    xtrack.k = k_tmp;

    double lat_tmp, lon_tmp, hae_tmp;
    fromECEF(x, y, z, &lat_tmp, &lon_tmp, &hae_tmp);

    float course = calculateBearing(pos.lat,  pos.lon,  lat_tmp,  lon_tmp);
    return course;
}

float calc_course_to_wp_wrpt_Xtrack_error(struct wp_class wpA, struct wp_class wpB, struct wp_class pos) {
   // Calcs cc to stear with respect to rhumb line correcton.
   // Rhumb line is from wpA -> wpB
   // if xtrack.k = 0.0 : return course to wpB
   // if xtrack.k = 1.0 : return course straigth to nearest pont on rhumb-line
   wp_class wpC;
   wp_class wpD;

   if (xtrack.type == STATIC)
       {
       if (xtrack.k<0.01) {
           wpD = wpB; // Point to steer towards
       } else {
           wpC = get_closet_point_on_rhumbLine(wpA,wpB, pos); // Closest point on Rhumb-line
           wpD.lat = wpB.lat*(1-xtrack.k)+wpC.lat*xtrack.k;   // Point to steer towards
           wpD.lon = wpB.lon*(1-xtrack.k)+wpC.lon*xtrack.k;
       }
   }
   else if (xtrack.type == DYNAMIC)
       {
           float XTE, L;
           wpC = get_closet_point_on_rhumbLine(wpA,wpB, pos); // Closest point on Rhumb-line
           // Calculate XTE
           XTE = calculateDistance(pos.lat, pos.lon, wpC.lat, wpC.lon);
           // Calculate distance between A and B
           L = calculateDistance(wpA.lat, wpA.lon, wpB.lat, wpB.lon);
           xtrack.k = (XTE > xtrack.S) ? 1 : 1 - xtrack.S/L;
           wpD.lat = wpB.lat*(1-xtrack.k)+wpC.lat*xtrack.k;   // Point to steer towards
           wpD.lon = wpB.lon*(1-xtrack.k)+wpC.lon*xtrack.k;
       }
   else return -1;

   float course = calculateBearing(pos.lat, pos.lon, wpD.lat, wpD.lon);
   return course;
}

//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

// Makes sure cog is in the range 0-2pi
float unwrap_2pi(float x)
{
   while (x < 0.0){ x = x + 2*pi; }
   while (x > 2.0*pi) { x = x - 2.0*pi; }
   return x;
}
//-------------------------------------------------------------------------------
// Makes sure cog is in the range -pi to pi
float unwrap_pi(float x)
{
   while (x < -pi) { x = x + 2.0*pi; }
   while (x >  pi) { x = x - 2.0*pi; }
   return x;
}
// -------------------------------------------------------------------------------
struct CCD distance_and_bearing(struct wp_class wp1, struct wp_class wp2){
  // Calculate distance in [NM]
  double dist   = 0.0;
  double course = 0.0;
  double Radie = 6378.137; //  [km] WGS84

  if ((wp2.lat!=wp1.lat) || (wp2.lon!=wp1.lon)) {
     double dlat = (wp2.lat-wp1.lat);
     if (abs(dlat)<0.000001){
        wp2.lat =wp2.lat + 0.000001;
        dlat = (wp2.lat-wp1.lat);
     }
     double dlon = wp2.lon-wp1.lon;
     double dfi  = log(tan(wp2.lat/2 + pi/4)/tan(wp1.lat/2 + pi/4));
     double q    = dlat/dfi;
     dist       = (float) (Radie/1.852*sqrt( dlat*dlat + q*q*dlon*dlon));
     course = (float) atan2( dlon, dfi);
     course = unwrap_2pi(course);
  }
  CCD ccd={(float)course,(float)dist};
  return ccd;
}

float calculateDistance(double lat1,double lon1,double lat2,double lon2)
{
    // Using Haversine formula
    double R = 6378127; // metres
    double phi1 = lat1;// .toRadians();
    double phi2 = lat2;// .toRadians();
    double dphi = (lat2-lat1);// .toRadians();
    double dlambda = (lon2-lon1);// .toRadians();

    double a = sin(dphi/2) * sin(dphi/2) +
            cos(phi1) * cos(phi2) *
            sin(dlambda/2) * sin(dlambda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return (float)(R * c);
}

// http://www.movable-type.co.uk/scripts/latlong.html
float calculateBearing(double lat1,double lon1,double lat2,double lon2)
{
    double y = sin(lon2-lon1) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
    double brng = atan2(y,x);

    return unwrap_2pi((float)brng);
}

//-------------------------------------------------------------------------------
wp_class get_closet_point_on_rhumbLine(struct wp_class A, struct wp_class B,  struct wp_class P){
    // Returns the point on line A-B that lies closest to P
    wp_class AP;
    AP.lat = P.lat - A.lat;
    AP.lon = P.lon - A.lon;
    wp_class AB;
    AB.lat = B.lat - A.lat;
    AB.lon = B.lon - A.lon;

    double ab2   = AB.lat*AB.lat + AB.lon*AB.lon;
    double ap_ab = AP.lat*AB.lat + AP.lon*AB.lon;
    double t     = ap_ab / ab2;

    if (t < 0.0) {t = 0.0;} // Takes care of overshoot at A
    if (t > 1.0) {t = 1.0;} // Takes care of overshoot at B

    wp_class closest_point;
    closest_point.lat = A.lat + (AB.lat * t);
    closest_point.lon = A.lon + (AB.lon * t);
    return closest_point;
}
