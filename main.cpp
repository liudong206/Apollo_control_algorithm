#include <malloc.h>
#include "control/mpc_controller.h"
#include "trajectory/trajectory.h"
#include "control/control.h"

using namespace control;
int main(int argc, char *argv[])
{
    //generate refrance trajectory
   TrajrefParams trajref_params;
   double des_linear_v = 20.0;//mps
   Trajectory traj;
   std::vector<TrajectoryPoint> planning_trajectory;
   traj.generate_trajref(TrajrefType::BigCircle,des_linear_v,
                         trajref_params,planning_trajectory);
    //init params
   SimpleDebug *debug = new SimpleDebug;
   VehicleParams VehPar;
   VehicleState veh_state(0.0,2.0,0.0,10);//x,y,heading,v
   int counter = 0;
   debug->time_step = 0.01;//仿真步长
   double distance_vehicle = 0;
   double simulation_distance = 180;//仿真距离
   //controller
   control::controller control;
   while (distance_vehicle < simulation_distance ) {
       counter++;
       debug->heading = veh_state.heading();
       debug->vehicle_x = veh_state.x();
       debug->vehicle_y = veh_state.y();
       distance_vehicle = distance_vehicle + veh_state.linear_velocity() * debug->time_step;
       control.LatAndLonController(&planning_trajectory,debug,veh_state);
       veh_state.update(veh_state,debug->acceleration_cmd,
                        debug->frant_wheel_delta,debug->time_step,VehPar.Wheelbase());//update vehicle state

       std::cout<<"counter:"<<counter<<std::endl;
       std::cout<<"veh_state.x:"<<veh_state.x()<<std::endl;
       std::cout<<"veh_state.y:"<<veh_state.y()<<std::endl;
       std::cout<<"veh_state.acc:"<<debug->acceleration_cmd<<std::endl;
       std::cout<<"veh_state.fran_wheel_delta:"<<debug->frant_wheel_delta<<std::endl;
   }
   delete debug;
   return 0;
}
