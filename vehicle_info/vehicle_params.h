#ifndef VEHICLE_PARAMS_H
#define VEHICLE_PARAMS_H
enum class VehicleType {
                         Q_EV,
                         OTHER };

class VehicleParams
{
public:
    VehicleParams();

    ~VehicleParams();

    //vehicle type
    VehicleType type() const;

    //vehicle length(m)
    double length() const;

    //vehicle width(m)
    double width() const;

    //vehicle Wheelbase(m)
    double Wheelbase() const;

    //vehicle front suspension length(m)
    double front_suspension_length() const;

    //Vehicle rear suspension length(m)
    double rear_suspension_length() const;

    // vehicle front wheels axle length(m)
    double front_axle_length() const;

    // vehicle rear wheels axle length(m)
    double rear_axle_length() const;

    //vehicle minimum turning radius of the midpoint of the rear axle(m)
    double minTurningRadius() const;

    //vehicle maximum steering wheel angle of the vehicle(degree)
    double maxSteeringAngle() const;

    //vehicle maximum steering wheel angle velocity of the vehicle(degree)
    double maxSteeringAngleVelocity() const;

    //vehicle maximum front wheel angle velocity of the vehicle(rad/s)
    double maxFrontWheelAngleVelocity() const;

    // Vehicle steering wheel ratio
    double steeringRatio() const;

    // Vehicle steering relative to inside wheel ratio
    double steeringRatio_inside() const;

    // Vehicle steering relative to outside wheel ratio
    double steeringRatio_outside() const;

    // Vehicle steering wheel ratio
    double steerRatio_ahead() const;

    // Vehicle steering relative to inside wheel ratio
    double steerRatio_inside_ahead() const;

    // Vehicle steering relative to outside wheel ratio
    double steerRatio_outside_ahead() const;

    //The driving distance corresponds to each gear wheel pulse(m)
    double dist_gear_pulse() const;

    double dist_gear_pulse_front() const;

    double dist_gear_pulse_back() const;

    // The front wheel gear pulse num: [0, max_num_gear_pulse]
    int max_num_gear_pulse() const;

    void set_type(const VehicleType vehicle_type);

    void set_length(const double length);

    void set_width(const double width);

    void set_wheelbase(const double wheelbase);

    void set_front_suspension_length(const double front_suspension_length);

    void set_rear_suspension_length(const double rear_suspension_length);

    void set_front_axle_length(const double front_axle_length);

    void set_rear_axle_length(const double rear_axle_length);

    void set_minTurningRadius(const double minTurningRadius);

    void set_maxSteeringAngle(const double maxSteeringAngle);

    void set_maxSteeringAngleVelocity(const double maxSteeringAngleVelocity);

    void set_steeringRatio(const double steeringRatio);

    void set_steeringRatio_inside(const double steeringRatioInSide);

    void set_steeringRatio_outside(const double steeringRatioOutSide);

    void set_steerRatio_ahead(const double steeringRatio);

    void set_steerRatio_inside_ahead(const double steeringRatio);

    void set_steerRatio_outside_ahead(const double steeringRatio);

    void set_dist_gear_pulse(const double dist_gear_pulse);

    void set_max_num_gear_pulse(const int max_num_gear_pulse);

    void set_maxfrontwheelangle(const double maxfrontwheelangle);

    double apa_radar_range_min = 0.0;
    double apa_radar_range_max = 0.0;
    double apa_radar_range_apha = 0.0;
    double upa_radar_range_min = 0.0;
    double upa_radar_range_max = 0.0;
    double upa_radar_range_apha = 0.0;

    double steerAngle_cmp_ = 0.0;
    double maxSteeringAngleLeft = 0.0;
    double maxSteeringAngleRight = 0.0;

private:
    VehicleType vehicle_type_;
    double length_ = 0.0;
    double width_ = 0.0;
    double wheelbase_ = 0.0;
    double front_suspension_length_ = 0.0;
    double rear_suspension_length_ = 0.0;
    double front_axle_length_ = 0.0;
    double rear_axle_length_ = 0.0;
    double minTurningRadius_ = 0.0;
    double maxSteeringAngle_ = 0.0;
    double maxSteeringAngleVelocity_ = 0.0;
    double steeringRatio_ = 0.0;
    double maxFrontWheelAngleVelocity_ = 0.0; //rad/s
    double steeringRatio_inside_ = 0.0;
    double steeringRatio_outside_ = 0.0;
    double steerRatio_ahead_ = 0.0; // 小蚂蚁前进与后退的传动比不一样
    double steerRatio_inside_ahead_ = 0.0; // 小蚂蚁前进与后退的传动比不一样
    double steerRatio_outside_ahead_ = 0.0; // 小蚂蚁前进与后退的传动比不一样
    double dist_gear_pulse_ = 0.0;
    double dist_gear_pulse_front_ = 0.0;
    double dist_gear_pulse_back_ = 0.0;
    int max_num_gear_pulse_ = 0.0;
    // init vehicle default parameters
    void init(VehicleType vehicle_type);
};


#endif // VEHICLE_PARAMS_H
