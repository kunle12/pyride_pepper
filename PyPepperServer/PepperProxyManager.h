/*
 *  PepperProxyManager.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 16/08/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef PepperProxyManager_h_DEFINED
#define PepperProxyManager_h_DEFINED
#include <string>
#include <pthread.h>
#include <boost/shared_ptr.hpp>

#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alaudioplayerproxy.h>
#include <alproxies/alaudiodeviceproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/alnotificationmanagerproxy.h>
#include <alproxies/alautonomouslifeproxy.h>
#include <alproxies/alanimatedspeechproxy.h>
#include <alproxies/alrechargeproxy.h>

namespace pyride {

using namespace AL;

typedef enum {
  WHITE = 0,
  BLANK,
  RED,
  BLUE,
  GREEN,
  YELLOW,
  PINK
} NAOLedColour;

enum { // pepper joints
  HEAD_YAW = 0,
  HEAD_PITCH,
  L_SHOULDER_PITCH,
  L_SHOULDER_ROLL,
  L_ELBOW_YAW,
  L_ELBOW_ROLL,
  L_WRIST_YAW,
  L_HAND,
  HIP_ROLL,
  HIP_PITCH,
  KNEE_PITCH,
  R_SHOULDER_PITCH,
  R_SHOULDER_ROLL,
  R_ELBOW_YAW,
  R_ELBOW_ROLL,
  R_WRIST_YAW,
  R_HAND
};

class PepperProxyManager
{
public:
  static PepperProxyManager * instance();
  ~PepperProxyManager();
  
  void initWithBroker( boost::shared_ptr<ALBroker> broker, boost::shared_ptr<ALMemoryProxy> memoryProxy );
  void sayWithVolume( const std::string & text, float volume = 1.0, bool toAnimate = true, bool toBlock = false );
  
  int loadAudioFile( const std::string & text );
  void unloadAudioFile( const int audioID );
  void unloadAllAudioFiles();
  void playWebAudio( const std::string & url );
  void playAudioID( const int audioID, bool toBlock = false );
  int  getAudioVolume();
  void setAudioVolume( const int vol );
  void pauseAudioID( const int audioID );
  void stopAllAudio();
  
  void setChestLED( const NAOLedColour colour );
  void pulsatingChestLED( const NAOLedColour colour1, const NAOLedColour colour2, const float period = 0.5 );
  void continuePluseChestLED();
  
  void getBatteryStatus( int & percentage, bool & isplugged, bool & ischarging, bool & isdischarging );

  bool getHeadPos( float & yaw, float & pitch );
  void moveHeadTo( const float yaw, const float pitch, bool absolute = false );
  void updateHeadPos( const float yaw, const float pitch, const float speed = 0.1 );

  void getBodyJointsPos( std::vector<float> & positions,
                        bool useSensor = false );
  void getArmJointsPos( bool isLeft, std::vector<float> & positions,
                       bool useSensor = false );
  void getLegJointsPos( std::vector<float> & positions,
                       bool useSensor = false );
  
  void setArmStiffness( bool isLeft, const float stiff );
  void setHeadStiffness( const float stiff );
  void setBodyStiffness( const float stiff );
  void setLegStiffness( const float stiff );

  bool moveArmWithJointPos( bool isLeft, const std::vector<float> & positions,
                           float frac_speed = 0.5 );
  
  void moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<float> > & trajectory,
                                                   std::vector<float> & times_to_reach, bool inpost = false );

  bool moveLegWithJointPos( const std::vector<float> & positions,
                           float frac_speed = 0.5 );

  bool moveBodyWithJointPos( const std::vector<float> & positions,
                            float frac_speed = 0.5 );

  void openHand( bool isLeft, float openRatio = 1.0, bool keepStiff = false );

  void stand( bool init = false );
  void crouch();

  void gotoStation();
  void leaveStation();

  bool moveBodyTo( const RobotPose & pose, float duration = 5.0, bool cancelPreviousMove = true );

  bool navigateBodyTo( const RobotPose & pose, bool cancelPreviousMove = true );

  void updateBodyPose( const RobotPose & pose );
  
  void timeoutCheck();

  void setAutonomousAbility( const std::string & ability, bool enable );

  void cancelBodyMovement();

  void fini();

private:
  static PepperProxyManager * s_pPepperProxyManager;

  boost::shared_ptr<ALTextToSpeechProxy> speechProxy_;
  boost::shared_ptr<ALAnimatedSpeechProxy> animateSpeechProxy_;
  boost::shared_ptr<ALMotionProxy> motionProxy_;
  boost::shared_ptr<ALRobotPostureProxy> postureProxy_;
  boost::shared_ptr<ALLedsProxy> ledProxy_;
  boost::shared_ptr<ALAudioPlayerProxy> audioPlayerProxy_;
  boost::shared_ptr<ALAudioDeviceProxy> audioDeviceProxy_;
  boost::shared_ptr<ALMemoryProxy> memoryProxy_;
  boost::shared_ptr<ALNavigationProxy> navigationProxy_;
  boost::shared_ptr<ALAutonomousLifeProxy> autoLifeProxy_;
  
  //motion related data
  ALValue jointLimits_;

  struct timeval cmdTimeStamp_;
  
  bool moveInitialised_;
  
  bool isChestLEDPulsating_;
  ALValue ledColourHex_;
  ALValue ledChangePeriod_;
  pthread_t runningThread_;
  pthread_t timeoutThread_;
  pthread_mutex_t t_mutex_;
  pthread_mutexattr_t t_mta;

  PepperProxyManager();
  
  float clamp( float val, int jointInd );
  int colour2Hex( const NAOLedColour colour );

};
} // namespace pyride
#endif // PepperProxyManager_h_DEFINED
