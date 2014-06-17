/*! @author Tomasz Kucner
 *  @date 14.01.2014
 *  @mainpage EPOS driver
 *
 *  This is a ROS wrapper for libepos. My aim was to create a
 *  general purpose wrpapper. However it was only tested only with the
 *  ALLO setup.
 */
#ifndef EPOS_DRIVER_HPP
#define EPOS_DRIVER_HPP
/* ROS HEADERS*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
/* CUSTOM HEADERS*/
#include "libepos/epos.h" 
/* BOOST HEADERS*/
#include <boost/thread/mutex.hpp> 
/* C/C++ HEADERS */
#include <string>
#include <sstream>
#include <signal.h>
/* MSG/SRV HEADERS */
#include <epos_driver/EPOSState.h>
#include <epos_driver/MoveTo.h>
#include <epos_driver/MoveCycle.h>

//                       ____       _                
//   ___ _ __   ___  ___|  _ \ _ __(_)_   _____ _ __ 
//  / _ \ '_ \ / _ \/ __| | | | '__| \ \ / / _ \ '__|
// |  __/ |_) | (_) \__ \ |_| | |  | |\ V /  __/ |   
//  \___| .__/ \___/|___/____/|_|  |_| \_/ \___|_|   
//      |_|                                          

/*! @brief Interface class for the servomotor controller
 *
 * This class impelents iterface for the epos servomotor
 * controller. It is based on libray epos.h.
 */
class eposdriver{
public:
  /*! @brief Configuration of the node */
  eposdriver(ros::NodeHandle parameters); 
  /* todo do we need explicite destructor?*/
  ~eposdriver(){};
  /*! @brief setup the servomotor*/
  int On();
  /*! @brief turn off the servomotor*/
  int Off();
  /*! @brief The main loop handling all the functionalities of the driver */
  int Main();
private:
  // todo add radPerTick constat
  ros::NodeHandle nodeHandler; ///< node handler
  ros::Publisher statePublisher;///< state publisher of EPOS servomotor 
  std::string port; ///< path to the epos device 
  bool synchronize; ///< keps the information if pose can be synchornized or not
  bool useRadps; ///< true if you want to use radians per second false if you want to use rpm 
  double pMaxVelocityRadps; ///< maximum velocity of the motor [rad/s] (prameter handeler) 
  double pAccelRadpss; ///< accelaeration ot the motor [rad/s^s] (prameter handeler) 
  double pDeccelRadpss; ///< decelleration ot the motor [rad/s^s] (prameter handeler) 
  int pMaxVelocityRpm; ///< maximum velocity of the motor [rpm] (prameter handeler) 
  int pAccelRpmps; ///< acceleration of the motor [rpm/s] (prameter handeler) 
  int pDeccelRpmps; ///< deceleration of the motor [rpm/s] (prameter handeler) 
  bool useTrapezoidal; ///< true if you want to use trapezoidal profile false for sinusoidal 
  unsigned int maxVelocity; ///< maximum velocity of the motor [rpm]
  unsigned int accel; ///< acceleration of the motor [rpm/s] 
  unsigned int deccel; ///< deceleration of the motor [rpm/s] 
  int motorState; ///< Motor on=1 off=0 
  //todo where is it used & why we need this?
  unsigned int moduleCount; ///< number of controlled modules 
  //todo where is it used & why we need this?
  double topicFrequency; ///< frequency at which the topic with state is published
  double highLimit; ///< the top limit of the cyclic motion of sensor
  double lowLimit; ///< the bottom limit of the cyclic motion of sensor
  bool moveDown;///< flag for direction of movement to low_limit
  bool moveUp;///< flag for direction of movement to high_limit
  bool moveSingle;///< flag for single movement
  std::string myFrameId;///< name of my coordination frame
  std::string parentFrameId;///< name of my parent coordination frame
  double sensorPoseX;///< x position of the wrist
  double sensorPoseY;///< y position of the wrist
  double sensorPoseZ;///< z position of the wrist
  int tresholdTicks;///< treshold for changing the direction of mobement
  int timeShift; ///< time which we need to shif to get proper time stamp of postion 

  /*! @brief Converts radians to rotations */
  unsigned int Radps2rpm(double radps);
  /*! @brief This function displays the error status */
  int EposError();
  /*! @brief This function displays the statueof epos */
  int EposState();
  /*! @brief This is a handler to a service for a single move*/
  bool MoveTo(epos_driver::MoveTo::Request &req,epos_driver::MoveTo::Response  &res);
   /*! @brief This is a handler to a service for a cyclic move*/
  bool MoveCycle(epos_driver::MoveCycle::Request &req,epos_driver::MoveCycle::Response &res);
};
#endif
