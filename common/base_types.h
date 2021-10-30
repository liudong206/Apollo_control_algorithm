#ifndef __BASE_TYPES_H_
#define __BASE_TYPES_H_


/**
 * @class PathPoint
 * @brief PathPoint is a data structure class
 *        for discretized path representation.
 */
class PathPoint {
 public:
  /// x coordinate
  double x = 0.0;
  /// y coordinate
  double y = 0.0;
  /// z coordinate
  double z = 0.0;
  /// derivative direction on the x-y plane
  double theta = 0.0;
  /// curvature on the x-y planning
  double kappa = 0.0;
  /// accumulated distance from beginning of the path
  double s = 0.0;
};

/**
 * @class TrajectoryPoint
 * @brief TrajectoryPoint is a data structure class
 *        for discretized trajectory representation.
 *        It inherits the variables from base class PathPoint.
 */
class TrajectoryPoint : public PathPoint {
 public:
  /// relative time from beginning of the trajectory
  double relative_time = 0.0;
  /// linear velocity
  double v = 0.0;
  /// linear acceleration
  double a = 0.0;
  /// curvature change rate w.r.t. time
  double dkappa = 0.0;
  /// directions defaut 1:forwarding  -1:reverse
  int directions = 1;
};

#endif /* __BASE_TYPES_H_ */
