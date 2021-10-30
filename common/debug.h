#ifndef DEBUG_H
#define DEBUG_H
struct SimpleDebug{
   double lateral_error;
   double ref_heading;
   double heading;
   double heading_error;
   double heading_error_rate;
   double lateral_error_rate;
   double curvature;
   double steer_angle;
   double steer_angle_feedforward;
   double steer_angle_feedback;
   double linear_velocity;
   double vehicle_x;
   double vehicle_y;
   double frant_wheel_delta;
   double trajref_x;
   double trajref_y;
   double time_step;

   double station_error;
   double speed_error;
   double acceleration;
   //LonDebug
   double lon_station_error;
   double station_error_limited;
   double preview_station_error;
   double speed_reference;
   double lon_speed_error;
   double preview_speed_reference;
   double preview_speed_error;
   double preview_acceleration_reference;
   double acceleration_cmd_closeloop;
   double slope_offset_compensation;
   double acceleration_cmd;
};

struct LonDebug{

};
#endif // DEBUG_H
