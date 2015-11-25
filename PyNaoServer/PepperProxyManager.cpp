/*
 *  PepperProxyManager.cpp
 *  PyPepperServer
 *
 *  Created by Xun Wang on 18/11/15.
 *  Copyright 2012, 2015 Galaxy Network. All rights reserved.
 *
 */
#include <sys/time.h>
#include <PyRideCommon.h>
#include "PepperProxyManager.h"

namespace pyride {

static const long kMotionCommandGapTolerance = 2 * 1000000 / kMotionCommandFreq;

PepperProxyManager * PepperProxyManager::s_pPepperProxyManager = NULL;

void * pulse_thread( void * controller )
{
  ((PepperProxyManager *)controller)->continuePluseChestLED();
  return NULL;
}

void * timeout_thread( void * controller )
{
  ((PepperProxyManager *)controller)->timeoutCheck();
  return NULL;
}

PepperProxyManager::PepperProxyManager() :
  moveInitialised_( false ),
  isChestLEDPulsating_( false ),
  runningThread_( (pthread_t)NULL ),
  timeoutThread_( (pthread_t)NULL )
{
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_, &t_mta );
}

PepperProxyManager::~PepperProxyManager()
{
  pthread_mutex_destroy( &t_mutex_ );
  pthread_mutexattr_destroy( &t_mta );
}

PepperProxyManager * PepperProxyManager::instance()
{
  if (!s_pPepperProxyManager) {
    s_pPepperProxyManager = new PepperProxyManager();
  }
  return s_pPepperProxyManager;
}

void PepperProxyManager::initWithBroker( boost::shared_ptr<ALBroker> broker, boost::shared_ptr<ALMemoryProxy> memoryProxy )
{
  memoryProxy_ = memoryProxy;

  try {
    speechProxy_ = boost::shared_ptr<ALTextToSpeechProxy>(new ALTextToSpeechProxy( broker ));
    speechProxy_->setLanguage( "English" );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALTextToSpeech.\n");
    speechProxy_.reset();
  }
  if (speechProxy_) {
    INFO_MSG( "Pepper text to speech is successfully initialised.\n" );
  }
  
  try {
    ledProxy_ = boost::shared_ptr<ALLedsProxy>(new ALLedsProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALLeds.\n");
    ledProxy_.reset();
  }
  if (ledProxy_) {
    INFO_MSG( "Pepper LED control is successfully initialised.\n" );
  }

  try {
    audioDeviceProxy_ = boost::shared_ptr<ALAudioDeviceProxy>(new ALAudioDeviceProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAudioDevice.\n");
    audioDeviceProxy_.reset();
  }
  if (audioDeviceProxy_) {
    INFO_MSG( "Pepper ALAudioDevice is successfully initialised.\n" );
  }

  try {
    audioPlayerProxy_ = boost::shared_ptr<ALAudioPlayerProxy>(new ALAudioPlayerProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAudioPlayer.\n");
    audioPlayerProxy_.reset();
  }
  if (audioPlayerProxy_) {
    INFO_MSG( "Pepper ALAudioPlayer is successfully initialised.\n" );
  }
  
  try {
    motionProxy_ = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALMotion.\n");
    motionProxy_.reset();
  }
  if (motionProxy_) {
    moveInitialised_ = false;
    AL::ALValue joints; // make sure we get joint limits in correct order
    jointLimits_.arraySetSize( 17 );
    jointLimits_[0] = motionProxy_->getLimits( "HeadYaw" )[0];
    jointLimits_[1] = motionProxy_->getLimits( "HeadPitch" )[0];
    jointLimits_[2] = motionProxy_->getLimits( "LShoulderPitch" )[0];
    jointLimits_[3] = motionProxy_->getLimits( "LShoulderRoll" )[0];
    jointLimits_[4] = motionProxy_->getLimits( "LElbowYaw" )[0];
    jointLimits_[5] = motionProxy_->getLimits( "LElbowRoll" )[0];
    jointLimits_[6] = motionProxy_->getLimits( "LWristYaw" )[0];
    jointLimits_[7] = motionProxy_->getLimits( "LHand" )[0];
    jointLimits_[8] = motionProxy_->getLimits( "HipRoll" )[0];
    jointLimits_[9] = motionProxy_->getLimits( "HipPitch" )[0];
    jointLimits_[10] = motionProxy_->getLimits( "KneePitch" )[0];
    jointLimits_[11] = motionProxy_->getLimits( "RShoulderPitch" )[0];
    jointLimits_[12] = motionProxy_->getLimits( "RShoulderRoll" )[0];
    jointLimits_[13] = motionProxy_->getLimits( "RElbowYaw" )[0];
    jointLimits_[14] = motionProxy_->getLimits( "RElbowRoll" )[0];
    jointLimits_[15] = motionProxy_->getLimits( "RWristYaw" )[0];
    jointLimits_[16] = motionProxy_->getLimits( "RHand" )[0];

    INFO_MSG( "Pepper Motion is successfully initialised.\n" );
  }
  
  try {
    postureProxy_ = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALRobotPosture.\n");
    postureProxy_.reset();
  }
  if (postureProxy_) {
    postureProxy_->goToPosture( "Stand", 0.6 );
    if (motionProxy_) {
      motionProxy_->rest();
    }
    INFO_MSG( "Pepper Robot Posture is successfully initialised.\n" );
  }

  try {
    navigationProxy_ = boost::shared_ptr<ALNavigationProxy>(new ALNavigationProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALNavigationProxy.\n");
    navigationProxy_.reset();
  }
  if (navigationProxy_) {
    INFO_MSG( "Pepper navigation is successfully initialised.\n" );
  }
}

void PepperProxyManager::sayWithVolume( const std::string & text, float volume, bool toBlock )
{
  if (speechProxy_ && text.length()) {
    if (volume <= 1.0 && volume > 0.0) {
      speechProxy_->setVolume( volume );
    }
    if (toBlock) {
      speechProxy_->say( text );
    }
    else {
      speechProxy_->post.say( text );
    }
  }
}

bool PepperProxyManager::getHeadPos( float & yaw, float & pitch )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    std::vector<float> curHeadPos = motionProxy_->getAngles( names, true );
    yaw = curHeadPos.at( 0 );
    pitch = curHeadPos.at( 1 );
    return true;
  }
  return false;
}
  
void PepperProxyManager::moveHeadTo( const float yaw, const float pitch, bool absolute )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    AL::ALValue stiff = 1.0f;
    AL::ALValue newHeadPos;

    newHeadPos.arraySetSize( 2 );

    if (absolute) {
      newHeadPos[0] = clamp( yaw, HEAD_YAW );
      newHeadPos[1] = clamp( pitch, HEAD_PITCH );
    }
    else {
      std::vector<float> curHeadPos;
      
      curHeadPos = motionProxy_->getAngles( names, true );
      
      newHeadPos[0] = clamp( yaw + curHeadPos.at( 0 ), HEAD_YAW );
      newHeadPos[1] = clamp( pitch + curHeadPos.at( 1 ), HEAD_PITCH );
    }
    motionProxy_->setStiffnesses( names, stiff );

    try { 
      motionProxy_->angleInterpolationWithSpeed( names, newHeadPos, 0.05 );
    }
    catch (...) {
      ERROR_MSG( "Unable to set angle interpolation to %s", newHeadPos.toString().c_str() );
    }
  }
}

void PepperProxyManager::updateHeadPos( const float yaw, const float pitch, const float speed )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    AL::ALValue newHeadPos;
    AL::ALValue stiff = 1.0f;

    newHeadPos.arraySetSize( 2 );

    std::vector<float> curHeadPos;
    motionProxy_->setStiffnesses( names, stiff );

    curHeadPos = motionProxy_->getAngles( names, true );

    newHeadPos[0] = clamp( yaw + curHeadPos.at( 0 ), HEAD_YAW );
    newHeadPos[1] = clamp( pitch + curHeadPos.at( 1 ), HEAD_PITCH );

    float myspeed = (speed > 1.0 || speed < 0.0 ) ? 0.1 : speed; // default to 0.1
    try {
      motionProxy_->setAngles( names, newHeadPos, myspeed );
    }
    catch (...) {
      ERROR_MSG( "Unable to change angles to %s", newHeadPos.toString().c_str() );
    }
  }
}

void PepperProxyManager::setHeadStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Head";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::setBodyStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Body";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::stand( bool init )
{
  if (postureProxy_) {
    postureProxy_->goToPosture( init ? "StandInit" : "Stand", 0.7 );
    moveInitialised_ = false;
  }
}

void PepperProxyManager::crouch()
{
  if (postureProxy_) {
    postureProxy_->goToPosture( "Crouch", 0.7 );
    motionProxy_->rest();
    moveInitialised_ = false;
  }
}

bool PepperProxyManager::moveBodyTo( const RobotPose & pose, float duration, bool cancelPreviousMove )
{
  if (!motionProxy_) {
    ERROR_MSG( "Unable to initialise walk." );
    return false;
  }
  if (motionProxy_->moveIsActive()) {
    if (cancelPreviousMove) {
      motionProxy_->stopMove();
    }
    else {
      ERROR_MSG( "Unable to issue new work command, robot is moving." );
      return false;
    }
  }
  else if (!moveInitialised_) {
    motionProxy_->wakeUp();
    motionProxy_->moveInit();
    moveInitialised_ = true;
  }
  motionProxy_->post.moveTo( pose.x, pose.y, pose.theta, duration );
  return true;
}

void PepperProxyManager::cancelBodyMovement()
{
  if (motionProxy_ && motionProxy_->moveIsActive()) {
    motionProxy_->stopMove();
  }
}

bool PepperProxyManager::navigateBodyTo( const RobotPose & pose, bool cancelPreviousMove )
{
  if (!navigationProxy_) {
    ERROR_MSG( "Unable to initialise navigation." );
    return false;
  }
  if (motionProxy_->moveIsActive()) {
    if (cancelPreviousMove) {
      motionProxy_->stopMove();
    }
    else {
      ERROR_MSG( "Unable to issue new work command, robot is moving." );
      return false;
    }
  }
  return navigationProxy_->navigateTo( pose.x, pose.y );
}

void PepperProxyManager::updateBodyPose( const RobotPose & pose )
{
  if (!motionProxy_)
    return;

  gettimeofday( &cmdTimeStamp_, NULL );

  if (!moveInitialised_) {
    motionProxy_->wakeUp();
    motionProxy_->moveInit();
    moveInitialised_ = true;
  }
  motionProxy_->moveToward( pose.x, pose.y, pose.theta );
  if (!timeoutThread_) {
    if (pthread_create( &timeoutThread_, NULL, timeout_thread, this ) ) {
      ERROR_MSG( "Unable to create thread to check motion command expiry.\n" );
      timeoutThread_ = (pthread_t)NULL;
      return;
    }
  }
}

void PepperProxyManager::getBodyJointsPos( std::vector<float> & positions,
                      bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = "Body";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::getArmJointsPos( bool isLeft, std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::getLegJointsPos( std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = "Leg";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::setArmStiffness( bool isLeft, const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::setLegStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Leg";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

bool PepperProxyManager::moveArmWithJointPos( bool isLeftArm, const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_)
    return false;
  
  int pos_size = positions.size();
  if (pos_size != 5) {
    return false;
  }
  
  //AL::ALValue names = isLeftArm ? "LArm" : "RArm";
  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll",
                                                      "LWristYaw") :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll",
                                                      "RWristYaw");
  
  AL::ALValue angles;

  motionProxy_->setStiffnesses( names, 1.0 );

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], (isLeftArm ? L_SHOULDER_PITCH + i : R_SHOULDER_PITCH + i) );
  }
  try {
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;
}

void PepperProxyManager::moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<float> > & trajectory,
                                                 std::vector<float> & times_to_reach, bool inpost )
{
  if (!motionProxy_)
    return;
  
  size_t traj_size = trajectory.size();
  
  if (traj_size <=0 || traj_size != times_to_reach.size())
    return;

  //AL::ALValue names = isLeftArm ? "LArm" : "RArm";
  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll" ) :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll" );
  AL::ALValue joints;
  AL::ALValue times;

  motionProxy_->setStiffnesses( names, 1.0 );

  joints.arraySetSize( 4 );
  times.arraySetSize( 4 );
  
  for (size_t j = 0; j < 4; ++j) {
    AL::ALValue angles;
    AL::ALValue ttr;
    
    angles.arraySetSize( traj_size );
    ttr.arraySetSize( traj_size );
    
    joints[j] = angles;
    times[j] = ttr;

    float time_to_reach_for_pt = 0.0;
    for (size_t jp = 0; jp < traj_size; ++jp) {
      joints[j][jp] = clamp( trajectory[jp][j], (isLeftArm ? L_SHOULDER_PITCH + j : R_SHOULDER_PITCH + j) );;
      time_to_reach_for_pt += times_to_reach[jp];
      times[j][jp] = time_to_reach_for_pt;
    }
  }
  try {
    if (inpost) {
      motionProxy_->post.angleInterpolation( names, joints, times, true );
    }
    else {
      motionProxy_->angleInterpolation( names, joints, times, true );
    }
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle interpolation to %s", joints.toString().c_str() );
  }
}

bool PepperProxyManager::moveLegWithJointPos( const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_)
    return false;

  int pos_size = positions.size();
  if (pos_size != 3) {
    return false;
  }
  AL::ALValue names = "Leg";
  AL::ALValue angles;
  
  motionProxy_->setStiffnesses( names, 1.0 );

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], HIP_ROLL + i );
  }
  try {
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;  
}

bool PepperProxyManager::moveBodyWithJointPos( const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_)
    return false;
  
  int pos_size = positions.size();
  if (pos_size != jointLimits_.getSize()) {
    return false;
  }
  AL::ALValue names = "Body";
  AL::ALValue angles;

  motionProxy_->setStiffnesses( names, 1.0 );
  
  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], i );
  }
  try {
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;  
}

int PepperProxyManager::loadAudioFile( const std::string & text )
{
  int audioID = -1;
  if (audioPlayerProxy_) {
    try {
      audioID = audioPlayerProxy_->loadFile( text );
    } catch (const std::exception &error) {
      ERROR_MSG( "Unable to load audio %s.\n", text.c_str() );
    }
  }
  return audioID;
}

void PepperProxyManager::unloadAudioFile( const int audioID )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadFile( audioID );
  }
}

void PepperProxyManager::unloadAllAudioFiles()
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadAllFiles();
  }
}

void PepperProxyManager::playWebAudio( const std::string & url )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.playWebStream( url, 1.0, 0.0 );
  }
}

void PepperProxyManager::playAudioID( const int audioID, bool toBlock )
{
  if (audioPlayerProxy_) {
    if (toBlock) {
      audioPlayerProxy_->play( audioID );
    }
    else {
      audioPlayerProxy_->post.play( audioID );
    }
  }
}

int PepperProxyManager::getAudioVolume()
{
  if (audioDeviceProxy_) {
    return audioDeviceProxy_->getOutputVolume();
  }
  else {
    return 0.0;
  }
}

void PepperProxyManager::setAudioVolume( const int vol )
{
  if (vol < 0 || vol > 100 )
    return;

  if (audioDeviceProxy_) {
    audioDeviceProxy_->setOutputVolume( vol );
  }
}

void PepperProxyManager::pauseAudioID( const int audioID )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.pause( audioID );
  }
}
void PepperProxyManager::stopAllAudio()
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.stopAll();
  }
}

void PepperProxyManager::setChestLED( const NAOLedColour colour )
{
  if (ledProxy_) {
    if (isChestLEDPulsating_) {
      isChestLEDPulsating_ = false;
      pthread_join( runningThread_, NULL );
    }
    switch (colour) {
      case WHITE:
        ledProxy_->on( "ChestLeds" );
        break;
      case BLANK:
        ledProxy_->off( "ChestLeds" );
        break;
      case RED:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case BLUE:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case GREEN:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case YELLOW:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case PINK:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      default:
        break;
    }
  }
}

void PepperProxyManager::pulsatingChestLED( const NAOLedColour colour1, const NAOLedColour colour2, const float period )
{
  if (!ledProxy_)
    return;

  
  pthread_mutex_lock( &t_mutex_ );
  
  ledColourHex_.arraySetSize( 2 );
  ledColourHex_[0] = this->colour2Hex( colour1 );
  ledColourHex_[1] = this->colour2Hex( colour2 );
  ledChangePeriod_.arraySetSize( 2 );
  ledChangePeriod_[0] = ledChangePeriod_[1] = period;

  pthread_mutex_unlock( &t_mutex_ );

  if (isChestLEDPulsating_)
    return;
  
  isChestLEDPulsating_ = true;

  if (pthread_create( &runningThread_, NULL, pulse_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to pulse LED.\n" );
    return;
  }
}

void PepperProxyManager::getBatteryStatus( int & percentage, bool & isplugged, bool & ischarging, bool & isdischarging )
{
  if (memoryProxy_) {
    percentage = memoryProxy_->getData( "BatteryChargeChanged" );
    isplugged = memoryProxy_->getData( "BatteryPowerPluggedChanged" );
    ischarging = memoryProxy_->getData( "BatteryChargingFlagChanged" );
    isdischarging = memoryProxy_->getData( "BatteryDisChargingFlagChanged" );
  }
}
void PepperProxyManager::gotoStation()
{
  // to be implemented
}

void PepperProxyManager::leaveStation()
{
  // to be implemented
}

void PepperProxyManager::fini()
{
  if (isChestLEDPulsating_) {
    isChestLEDPulsating_ = false;
    pthread_join( runningThread_, NULL );
  }
  if (speechProxy_) {
    speechProxy_.reset();
  }
  if (motionProxy_) {
    motionProxy_.reset();
  }
  if (ledProxy_) {
    ledProxy_.reset();
  }
  if (audioDeviceProxy_) {
    audioDeviceProxy_.reset();
  }
  if (audioPlayerProxy_) {
    audioPlayerProxy_.reset();
  }
}

// helper function
void PepperProxyManager::continuePluseChestLED()
{
  while (isChestLEDPulsating_) {
    pthread_mutex_lock( &t_mutex_ );
    ledProxy_->fadeListRGB( "ChestLeds", ledColourHex_, ledChangePeriod_ );
    pthread_mutex_unlock( &t_mutex_ );

    usleep( 10000 );
  }
}

void PepperProxyManager::timeoutCheck()
{
  struct timeval now;
  
  do {
    usleep( 10000 );
    gettimeofday( &now, NULL );
  } while (((now.tv_sec - cmdTimeStamp_.tv_sec) * 1000000 + now.tv_usec - cmdTimeStamp_.tv_usec) < kMotionCommandGapTolerance);
  this->cancelBodyMovement();
  timeoutThread_ = (pthread_t)NULL;
}
  
float PepperProxyManager::clamp( float val, int jointInd )
{
  if (jointInd >= jointLimits_.getSize()) {
    ERROR_MSG( "invalid joint index %d\n", jointInd );
    return val;
  }
  
  if (val < (float)jointLimits_[jointInd][0]) {
    return (float)jointLimits_[jointInd][0];
  }
  else if (val > (float)jointLimits_[jointInd][1]) {
    return (float)jointLimits_[jointInd][1];
  }
  else {
    return val;
  }
}

int PepperProxyManager::colour2Hex( const NAOLedColour colour )
{
  int retval = 0;
  switch (colour) {
    case WHITE:
      retval = 0x00ffffff;
      break;
    case RED:
      retval = 0x00ff0000;
      break;
    case BLUE:
      retval = 0x000000ff;
      break;
    case GREEN:
      retval = 0x0000ff00;
      break;
    case YELLOW:
      retval = 0x00ffff00;
      break;
    case PINK:
      retval = 0x00ff00ff;
      break;
    default:
      break;
  }
  return retval;
}
} // namespace pyride
