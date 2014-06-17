#include "epos_driver/epos_driver.hpp"

//                       ____       _                
//   ___ _ __   ___  ___|  _ \ _ __(_)_   _____ _ __ 
//  / _ \ '_ \ / _ \/ __| | | | '__| \ \ / / _ \ '__|
// |  __/ |_) | (_) \__ \ |_| | |  | |\ V /  __/ |   
//  \___| .__/ \___/|___/____/|_|  |_| \_/ \___|_|   
//      |_|                                          
/*! In the constructor we set all the initial parmaeters and we also
 *  start the node itself however we do not start the communication
 *  with the servomotor. 
 * 
 * @param[in] param thorugh this NodeHandle there are passed
 * argumnets to the node.
 *
 * @see On  
 */

eposdriver::eposdriver(ros::NodeHandle parameters){
  parameters.param<std::string>("port",port,"/dev/usb0");
  parameters.param<bool>("use_radps",useRadps,true);  
  parameters.param<double>("max_velocity_radps",pMaxVelocityRadps,0.4);
  parameters.param<double>("accel_radpss",pAccelRadpss,0.02);
  parameters.param<double>("deccel_radpss",pDeccelRadpss,0.02);
  parameters.param<int>("max_velocity_rpm",pMaxVelocityRpm,57);
  parameters.param<int>("accel_rpmps",pAccelRpmps,2);
  parameters.param<int>("deccel_rpmps",pDeccelRpmps,2);
  parameters.param<bool>("use_trapezoidal",useTrapezoidal,false);
  parameters.param<double>("topic_frequency",topicFrequency,20);
  parameters.param<std::string>("my_frame_id", myFrameId, "maxon");
  parameters.param<std::string>("parent_frame_id", parentFrameId, "base");
  parameters.param<double>("sensor_pose_x", sensorPoseX, 0.0);
  parameters.param<double>("sensor_pose_y", sensorPoseY, 0.0);
  parameters.param<double>("sensor_pose_z", sensorPoseZ, 0.0);
  parameters.param<int>("treshold_ticks",tresholdTicks,10);
  parameters.param<int>("time_shift",timeShift,6); //ms

  if(useRadps){
    maxVelocity=Radps2rpm(pMaxVelocityRadps);
    accel=Radps2rpm(pAccelRadpss);
    deccel=Radps2rpm(pAccelRadpss);
  }
  else{
    maxVelocity=pMaxVelocityRpm;
    accel=pAccelRpmps;
    deccel=pAccelRpmps;
  }
}

//   ___        
//  / _ \ _ __  
// | | | | '_ \ 
// | |_| | | | |
//  \___/|_| |_|
/*! In this function take place all the necessarry setup of the
 *  driver. It opens and configure serial port. 
 * 
 * This function is based on MainSetup from ALL4-e-Ham.
 *
 * @return If setup was correct returns 0 otherwise 1.
 */
// todo Is this return enough or should we return speciffic error code?


int eposdriver::On(){
  WORD eposStatus=0x0;
  int eposState=0;
  char deviceName[128];
  int functionReturns;
  unsigned long win;// position window
  long pos = -99;// position of the EPOS
  ROS_INFO("*** EPOS driver initialising ***");
  ROS_INFO("Openning the device. Port: %s",port.c_str());
  if(openEPOS(const_cast<char *>(port.c_str()))<0){
    ROS_ERROR("Could not open EPOS on port: %s.",port.c_str());
    ROS_ERROR("EXITING NODE");
    ros::shutdown();
  }
  else{
    ROS_INFO("EPOS on port %s opened.",port.c_str());
  }
  //todo Do we need to read the device name?
  if(readDeviceName(deviceName)<0){ //reading the name of the connected device
    ROS_ERROR("Could not read the device name");
  }
  else{
    ROS_INFO("Connected device name is: %s",deviceName);
  }
  //todo Do we need to read the software version?
  functionReturns=readSWversion();
  if(functionReturns<0){
    ROS_ERROR("Could not read the software version");
  }
  else{
    ROS_INFO("Software version is is: %x",functionReturns);
  }
  //todo Do we need to read the RS232 timeout?
  functionReturns=readRS232timeout();
  if(functionReturns<0){
    ROS_ERROR("Could not read the RS232 timeout");
    EposError();
  }
  else{
    ROS_INFO("RS232 timeout is: %d ms\n", functionReturns);
  }

  ROS_INFO("*** Checking EPOS status ***");
  functionReturns = readStatusword(&eposStatus);
  if (functionReturns<0){
    ROS_ERROR("Could not check EPOS status");
    EposError();
  }
  else{
    ROS_INFO("EPOS status is: %#06x \n",eposStatus);
  }
  ROS_INFO("*** Switching on EPOS ***");
  eposState=checkEPOSstate();
  ROS_INFO("EPOS state; %d:",eposState);
  if (eposState==11){
    ROS_INFO("EPOS is in FAULT state, doing FAULT RESET.");
    changeEPOSstate(6);// reset FAULT
    eposState=checkEPOSstate();// check status again
    if(eposState==11){
      ROS_ERROR("EPOS still in FAULT state, quit!");
      EposError();
      return -1;//exit(1);
    }
    else{
      ROS_INFO("Success! (Now in state %d)", eposState);
    }
  }
  if(eposState!=4 && eposState!=7) { // EPOS not running, issue a quick stop
    ROS_INFO("EPOS is in FAULT state, doing FAULT RESET.");
    changeEPOSstate(3);
    eposState=checkEPOSstate();// EPOS should now be in 'switch on disabled' (2)
    if (eposState!=2){
      ROS_ERROR("EPOS is NOT in 'switch on disabled' state, quit!");
      return -1; //exit(1);
    }
    else {  // EPOS is in 'switch on disabled'
      ROS_INFO("EPOS is in 'switch on disabled' state, doing shutdown.");
      changeEPOSstate(0); // issue a 'shutdown'
    }
    ROS_INFO("Switching on EPOS");
    changeEPOSstate(1);
    ROS_INFO("Enable operation" );
    changeEPOSstate(5);
  }
  EposState();
  readPositionWindow(&win);
  ROS_INFO("EPOS position window is %lu.", win);  
  readActualPosition(&pos);   // actual position
  ROS_INFO("Please wait until homing complete...");
  if (setHomePolarity(1)) {// filter wheel home switch is low-active; THIS IS NOT THE DEFAULT! what does it mean????
    ROS_ERROR(" *** UNABLE TO SET HomeSwitch TO low-active!!! ***");
  }
  while ((functionReturns= doHoming(18,0))!=0){
    ROS_ERROR("#### doHoming() returned %d ####", functionReturns);
    changeEPOSstate(5);
  }
  if(setOpMode(1)){ // Set Profile Position Mode once, to avoid having to dot at each call to moveAbsolute().
    ROS_ERROR("Error with setOpMode(1)");
    return(-1);
  }
  EposState();
  readActualPosition(&pos);
  ROS_INFO("EPOS position is %ld.", pos);
  motorState=1;
  moduleCount=1;
  ROS_INFO("Prameters %d %d %d", maxVelocity,accel,deccel);
  set_speed_profile(maxVelocity,accel,deccel,useTrapezoidal);
  ROS_INFO("EPOS driver is reday");
  return(0);
}

//   ___   __  __ 
//  / _ \ / _|/ _|
// | | | | |_| |_ 
// | |_| |  _|  _|
//  \___/|_| |_|  
/*! This function closes communication with the epos servomotor. */
int eposdriver::Off(){
  ROS_INFO("Shutting EPOS driver down");
  if(closeEPOS()<0){
    ROS_ERROR("Failure druing shooting down");
    return -1;
  }
  else{
    ROS_INFO("EPOS driver has been shutdown");
  }
  return 0;
}

//  __  __       _       
// |  \/  | __ _(_)_ __  
// | |\/| |/ _` | | '_ \ 
// | |  | | (_| | | | | |
// |_|  |_|\__,_|_|_| |_|
/*! This function handles all the functionalities of the node. All the
 *  topics are published and all services are advertised inside of
 *  this  function. 
 */
 // todo Check if all converions are corret.
 
int eposdriver::Main(){
  long rawPosition;
  double pos;
  long rawSpeed;
  double speed;
  double acceleration;
  short current;
  long oldSpeed;
  float deltaSpeed;
  bool validDeltaSpeed(false);
  moveSingle=true;
  synchronize=true;
  epos_driver::EPOSState msg;
  ros::Time then;
  ros::Time now;
  ros::Duration elapsedTime;
  static const double mradPerTick=0.000209;//1000.0*0.000209;
  //  ros::Rate loopRate(topicFrequency);
  statePublisher=nodeHandler.advertise<epos_driver::EPOSState>("EPOSState",1000);
  ros::ServiceServer MoveToService=nodeHandler.advertiseService("MoveTo",&eposdriver::MoveTo,this);
  ros::ServiceServer MoveCycleService=nodeHandler.advertiseService("MoveCycle",&eposdriver::MoveCycle,this);
  tf::Transform transform;
  static tf::TransformBroadcaster tfPublisher;

  ros::Time time_framerate_start;
  ros::Time time_framerate_end;
  ros::Time time_realtime_start;
  ros::Time time_realtime_end;
  
  time_framerate_start=ros::Time::now();
  time_realtime_start=time_framerate_start;

  while(ros::ok()){
    time_realtime_end=ros::Time::now();
    //  long real_elapsed=time_realtime_start.toNsec()-time_realtime_end.toNsec();
    time_realtime_start=ros::Time::now();

    oldSpeed=speed;
    then=now;
    now=ros::Time::now();
    readActualVelocity(&rawSpeed);
    readActualPosition(&rawPosition);
    readActualCurrent(&current);
    pos=mradPerTick*static_cast<double>(rawPosition);
    speed=mradPerTick*static_cast<double>(rawSpeed);
    if(validDeltaSpeed){
      deltaSpeed=speed-oldSpeed;
      elapsedTime=now-then;
      acceleration=deltaSpeed/elapsedTime.toSec(); 
    }
    else{
      acceleration=0;
    }
    if(!validDeltaSpeed)
      validDeltaSpeed=true;
    msg.header.stamp=ros::Time::now();
    msg.raw_position=rawPosition;
    msg.position=pos;
    msg.raw_speed=rawSpeed;
    msg.speed=speed;
    msg.acceleration=acceleration;
    msg.current=current;
    if(moveSingle){
      if(speed==0.0)
        synchronize=true;
      else
        synchronize=false;
    }
    if(!moveSingle){
      if(moveDown){
        synchronize=false;
        if(rawPosition>=int(highLimit/mradPerTick)-tresholdTicks){
          synchronize=true;
          moveAbsolute(lowLimit/mradPerTick);
          moveUp=true;
          moveDown=false;
        }
      }
      else if(moveUp){
        synchronize=false;
        if(rawPosition<=int(lowLimit/mradPerTick)+tresholdTicks){
          synchronize=true;
          moveAbsolute(highLimit/mradPerTick);
          moveUp=false;
          moveDown=true;
        }
      }
    }
    msg.sync=synchronize;
    transform.setOrigin(tf::Vector3(sensorPoseX,sensorPoseY,sensorPoseZ)); 
    transform.setRotation(tf::createQuaternionFromRPY(0.0,0.0,pos));
    tfPublisher.sendTransform(tf::StampedTransform(transform,ros::Time::now()+ros::Duration(0,timeShift), parentFrameId, myFrameId));   
    statePublisher.publish(msg);
    time_framerate_end=ros::Time::now();
    ros::Duration elapsed=time_framerate_end-time_framerate_start;
    ros::Duration sleep_;
    if (ros::Duration(1/topicFrequency)-elapsed<ros::Duration(0))
      sleep_= ros::Duration(0);
    else
      sleep_= ros::Duration(1/topicFrequency)-elapsed;
    ros::Duration(sleep_).sleep();
    time_framerate_start=ros::Time::now();
    ros::spinOnce();
    //  loopRate.sleep(); 
  }
  return 0;
}

//  ____           _     ____                       
// |  _ \ __ _  __| |___|___ \ _ __ _ __  _ __ ___  
// | |_) / _` |/ _` / __| __) | '__| '_ \| '_ ` _ \ 
// |  _ < (_| | (_| \__ \/ __/| |  | |_) | | | | | |
// |_| \_\__,_|\__,_|___/_____|_|  | .__/|_| |_| |_|
//                                 |_|              
/*! This function is used to convert the units of rotation speed. It
 *  is based on code from ALL-4-eHAM
 *
 * @param[in] radps input in radians
 * @return value in rpm
 */
 //todo check what is doing which variable
 
unsigned int eposdriver::Radps2rpm(double radps){
  static const double mradPerTick=1000.0*0.000209;
  static const double radpsToRpm = 3.0*10.0*100.0*1000.0/mradPerTick/1000.0;
  ROS_INFO("value %f -> %d",radps*radpsToRpm,int(radps*radpsToRpm));
  return int(radps*radpsToRpm);
}

//  _____                 _____                     
// | ____|_ __   ___  ___| ____|_ __ _ __ ___  _ __ 
// |  _| | '_ \ / _ \/ __|  _| | '__| '__/ _ \| '__|
// | |___| |_) | (_) \__ \ |___| |  | | | (_) | |   
// |_____| .__/ \___/|___/_____|_|  |_|  \___/|_|   
//       |_|                                        
/*! This function displays error of epos driver in more ROS friendly
 *  way. It is just a rewritten function checkEPOSerror().
 *
 * @see checkEPOSerror
 */
int eposdriver::EposError(){
  switch(E_error) {
    case E_NOERR: 
      return(0);
    case E_ONOTEX:
      ROS_ERROR("EPOS responds with error: requested object does not exist!"); break;
    case E_SUBINEX:
      ROS_ERROR("EPOS responds with error: requested subindex does not exist!"); break;
    case E_OUTMEM:
      ROS_ERROR("EPOS responds with error: out of memory!"); break;
    case E_NOACCES:
      ROS_ERROR("EPOS responds with error: unsupported access to an object!"); break;
    case E_WRITEONLY:
      ROS_ERROR("EPOS responds with error: attempt to read a write-only object!"); break;
    case E_READONLY:
      ROS_ERROR("EPOS responds with error: attempt to write a read-only object!"); break;
    case E_PARAMINCOMP:
      ROS_ERROR("EPOS responds with error: general parameter incompatibility!"); break;
    case E_INTINCOMP:
      ROS_ERROR("EPOS responds with error: general internal incompatibility in the device!"); break;
    case E_HWERR:
      ROS_ERROR("EPOS responds with error: access failed due to an HARDWARE ERROR!"); break;
    case E_PRAGNEX:
      ROS_ERROR("EPOS responds with error: value range of parameter exeeded!"); break;
    case E_PARHIGH:
      ROS_ERROR("EPOS responds with error: value of parameter written is too high!"); break;
    case E_PARLOW:
      ROS_ERROR("EPOS responds with error: value of parameter written is too low!"); break;
    case E_PARREL:
      ROS_ERROR("EPOS responds with error: maximum value is less than minimum value!"); break;
    case E_NMTSTATE:
      ROS_ERROR("EPOS responds with error: wrong NMT state!"); break;
    case E_RS232:
      ROS_ERROR("EPOS responds with error: rs232 command illegeal!"); break;
    case E_PASSWD:
      ROS_ERROR("EPOS responds with error: password incorrect!"); break;
    case E_NSERV:
      ROS_ERROR("EPOS responds with error: device not in service mode!"); break;
    case E_NODEID:
      ROS_ERROR("EPOS responds with error: error in Node-ID!"); break;
    default:
      ROS_ERROR("EPOS responds with error: unknown EPOS error code: %#lx",E_error); break;
  }
  return(-1);
}

//  _____                 ____  _        _       
// | ____|_ __   ___  ___/ ___|| |_ __ _| |_ ___ 
// |  _| | '_ \ / _ \/ __\___ \| __/ _` | __/ _ \
// | |___| |_) | (_) \__ \___) | || (_| | ||  __/
// |_____| .__/ \___/|___/____/ \__\__,_|\__\___|
//       |_|                                     
/*! This function displays state of epos driver in more ROS friendly
 *  way. It is just a rewritten function printEPOSstate().
 *
 * @see printEPOSstate
 */
int eposdriver::EposState(){
  switch(checkEPOSstate()){
    case 0: ROS_INFO("EPOS is in state: start"); break;
    case 1: ROS_INFO("EPOS is in state: Not ready to switch on."); break;
    case 2: ROS_INFO("EPOS is in state: Switch on disabled."); break;
    case 3: ROS_INFO("EPOS is in state: Ready to switch on."); break;
    case 4: ROS_INFO("EPOS is in state: Switched on."); break;
    case 5: ROS_INFO("EPOS is in state: Refresh."); break;
    case 6: ROS_INFO("EPOS is in state: Measure init."); break;
    case 7: ROS_INFO("EPOS is in state: Operation enable."); break;
    case 8: ROS_INFO("EPOS is in state: Quick stop active"); break;
    case 9: ROS_INFO("EPOS is in state: Fault reaction active (disabled)"); break;
    case 10: ROS_INFO("EPOS is in state: Fault reaction active (enabled)"); break;
    case 11: ROS_INFO("EPOS is in state: FAULT\n"); break;
    default: 
      ROS_INFO("EPOS is in state: UNKNOWN!\n");
      return(-1);
  }
  return(0);
}

//  __  __               _____     
// |  \/  | _____   ____|_   _|__  
// | |\/| |/ _ \ \ / / _ \| |/ _ \ 
// | |  | | (_) \ V /  __/| | (_) |
// |_|  |_|\___/ \_/ \___||_|\___/ 
/*! This is a handler for for the servie MoveTo. It accepts the input
 *  in radians. This service moves the sensor only once to given
 *  position. 
 */
bool eposdriver::MoveTo(epos_driver::MoveTo::Request  &req,epos_driver::MoveTo::Response  &res){
  static const float radPerTick = 0.000209;
  moveAbsolute(req.pose/radPerTick);
  moveSingle=true;
  return true;
}

//  __  __                 ____           _      
// |  \/  | _____   _____ / ___|   _  ___| | ___ 
// | |\/| |/ _ \ \ / / _ \ |  | | | |/ __| |/ _ \
// | |  | | (_) \ V /  __/ |__| |_| | (__| |  __/
// |_|  |_|\___/ \_/ \___|\____\__, |\___|_|\___|
//                             |___/             
/*! This is a handler for for the servie MoveTo. It accepts the input
 *  in radians. This service moves the sensor only once to given
 *  position. 
 */
bool eposdriver::MoveCycle(epos_driver::MoveCycle::Request  &req,epos_driver::MoveCycle::Response &res){
  static const float radPerTick = 0.000209;
  long rawSpeed;
  highLimit=req.pose_top;
  lowLimit=req.pose_bottom;
  moveAbsolute(highLimit/radPerTick);
  if(highLimit>lowLimit){
    do{
      readActualVelocity(&rawSpeed);
      ros::Duration(0.1).sleep();
    }while(rawSpeed!=0);
    moveSingle=false;
    moveUp=false;
    moveDown=true;
  }
  else {
    ROS_ERROR("Limits of motion wrongly setted!");
  }
  return true;
}

