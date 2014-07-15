#ifndef CAMERA_CONSTRAINTS_H
#define CAMERA_CONSTRAINTS_H

struct CameraConstraints
{
  double height_min;
  double height_max;
  double pitch_min;
  double pitch_max;
  double hfov;
  double vfov;
  double range_min;
  double range_max;
  double roll;
  
  CameraConstraints()
  : height_min(0.0)
  , height_max(0.0)
  , pitch_min(0.0)
  , pitch_max(0.0)
  , hfov(0.0)
  , vfov(0.0)
  , range_min(0.0)
  , range_max(0.0)
  , roll(0.0)
  {}
};

#endif // CAMERA_CONSTRAINTS_H

