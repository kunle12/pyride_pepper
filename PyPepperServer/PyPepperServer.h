/*
 *  PyPepperServer.h
 *  PyPepperServer
 *
 *  Created by Xun Wang on 18/11/15.
 *  Copyright 2009,2015 Galaxy Network. All rights reserved.
 *
 */

#ifndef PyPepperServer_h_DEFINED
#define PyPepperServer_h_DEFINED

#include <vector>
#include <string>
#include <rttools/rttime.h>
#include <boost/shared_ptr.hpp>

#include <althread/almutex.h>
#include <alerror/alerror.h>
#include <alcommon/almodule.h>
#include <alvision/alimage.h>

#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almemoryproxy.h>
#include <althread/almutex.h>
#include <alaudio/alsoundextractor.h>

#include <ServerDataProcessor.h>

namespace pyride {

using namespace AL;

class NaoCam : public VideoDevice {
public:
  NaoCam( boost::shared_ptr<ALBroker> pBroker, const std::string & name, const int cameraID );
  ~NaoCam();

  bool initDevice();
  void finiDevice();

  void continueProcessing();

  bool setCameraParameter( int pid, int value );
  void getVideoSettings( VideoSettings & settings );
  void takeSnapshot( const VideoDeviceDataHandler * dataHandler );

private:
  boost::shared_ptr<ALVideoDeviceProxy> videoProxy_;
  boost::shared_ptr<ALBroker> broker_;

  pthread_t procThread_;
  pthread_attr_t threadAttr_;

  std::string gvmName_;

  bool takeSnapShot_;
  int cameraID_;

  bool getDefaultVideoSettings();

  bool initWorkerThread();
  void finiWorkerThread();
};

class PyPepperServer : public ALSoundExtractor, AudioDevice, PyRideExtendedCommandHandler {
public:
  PyPepperServer( boost::shared_ptr<ALBroker> pBroker, const std::string & pName );
  virtual ~PyPepperServer();

  bool innerTest() { return true; }
  bool isRunning() { return true; }
  std::string version() { return "2.4.3"; }

  // Automatically called right after the module is created.
  virtual void init();
  void fini();

  // callbacks
  void onRightBumperPressed();
  void onLeftBumperPressed();
  void onBackBumperPressed();

  void onFrontTactilTouched();
  void onMiddleTactilTouched();
  void onRearTactilTouched();

  void onRightHandBackTouched();
  void onLeftHandBackTouched();

  void onMovementFailed();

  void onSingleChestButtonPressed();
  void onDoubleChestButtonPressed();
  void onTripleChestButtonPressed();

  void onPowerHatchOpened();
  void onConnectedToChargingStation();
  void onBatteryNearlyEmpty();
  void onBatteryPowerPlugged();
  void onBatteryChargeChanged();

  // Audio device implementation
  bool initDevice();
  void finiDevice();
  // automatically called when sound detection is on
  void process( const int &pNbOfInputChannels, const int &pNbrSamples,
               const AL_SOUND_FORMAT *pDataInterleaved,
               const AL::ALValue &pTimeStamp );

private:
  boost::shared_ptr<ALMemoryProxy> memoryProxy_;
  boost::shared_ptr<ALMutex> callbackMutex_;

  AL_SOUND_FORMAT * audioBuffer_;

  VideoDeviceList pepperCams_;
  AudioDeviceList pepperAudio_;

  bool executeRemoteCommand( PyRideExtendedCommand command, int & retVal,
                            const unsigned char * optionalData = NULL,
                            const int optionalDataLength = 0 );
  void cancelCurrentOperation();
  void notifySystemShutdown();

  bool initWorkerThread();
  void finiWorkerThread();
};
} // namespace pyride

#endif // PyPepperServer_h_DEFINED
