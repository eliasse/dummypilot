#ifndef __STRUCTS__
#define __STRUCTS__

struct wp_class {
  double lat; // [rad]
  double lon; // [rad]
  double alt; // [m]
  float depth;

  bool operator==(struct wp_class& other) {
    return (this->lat == other.lat) && (this->lon == other.lon);
  }
};


#endif
