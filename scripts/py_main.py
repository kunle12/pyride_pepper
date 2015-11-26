import PyPepper
import math
import constants
import tinstate
import messenger
import tininfo
import time

from timers import timermanager

myMessenger = None

msgTryTimer = -1
calTryTimer = -1
cktime = time.time()

def userLogon( name ):
  PyPepper.say( '%s has logged on.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, False )

def userLogoff( name ):
  PyPepper.say( '%s has logged off.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, len(PyPepper.listCurrentUsers()) == 0)

def bumperActions( id ):
  global cktime
  if time.time() - cktime <= 60:
    return #ignore

  if id == 'right':
    users = PyPepper.listCurrentUsers()
    if len(users) == 0:
      PyPepper.say( 'No one is telepresent.' )
    else:
      PyPepper.say( 'Following users are telepresent:' )
      for i in users:
        PyPepper.say( '\pau=1000\ %s' % i )
  elif id == 'left':
    if len(PyPepper.listCurrentUsers()) > 0:
      PyPepper.say( 'I will notify the telepresent members to tune in.' )
      PyPepper.updateOperationalStatus( constants.CUSTOM_STATE, 'Need your attention' )

  cktime = time.time()

def remoteCommandActions( cmd, arg ):
  if cmd == constants.LEARN_POSE:
    if arg == "Stand":
      PyPepper.stand()
    if arg == "Stand (init)":
      PyPepper.stand(True)
    elif arg == "Crouch":
      PyPepper.crouch()
  elif cmd == constants.LEARN_OBJECT:
    PyPepper.takeCameraSnapshot()

def timerLapsedActions( id ):
  timermanager.onTimerLapsed( id )

def timerActions( id ):
  global myMessenger, msgTryTimer

  if msgTryTimer == id and myMessenger.checkin():
    PyPepper.removeTimer( msgTryTimer )
    msgTryTimer = -1
  else:
    timermanager.onTimerCall( id )

def chestBtnActions( id ):
  global myMessenger, purgeuser
  if id == 1:
    myMessenger.announce()
#do report messages
  elif id == 2:
    myMessenger.purgearchive()
  elif id == 3:
    PyPepper.say( constants.INTRO_TEXT )

def powerPlugChangeActions( isplugged ):
  global myMessenger

  text = ""
  if isplugged:
    text = "I'm on main power."
  else:
    text = "I'm on battery power."

  PyPepper.say( text )

  if myMessenger:
    myMessenger.updatestatus( text )

def batteryChargeChangeActions( batpc, isdischarge ):
  global myMessenger
  
  if batpc < 20 and isdischarge:
    PyPepper.say( "I'm low on battery, please put me back on main power." )

    if myMessenger:
      myMessenger.updatestatus( "I have only %d percent battery power left!" % batpc )

def systemShutdownActions():
  global myMessenger

  myMessenger.checkout()
  PyPepper.say( "I am going off line. Goodbye." )

def main():
  global myMessenger, msgTryTimer

  PyPepper.onUserLogOn = userLogon
  PyPepper.onUserLogOff = userLogoff
  PyPepper.onBumperPressed = bumperActions
  PyPepper.onTimer = timerActions
  PyPepper.onTimerLapsed = timerLapsedActions
  PyPepper.onRemoteCommand = remoteCommandActions
  PyPepper.onChestButtonPressed = chestBtnActions
  PyPepper.onSystemShutdown = systemShutdownActions
  PyPepper.onPowerPluggedChange = powerPlugChangeActions
  PyPepper.onBatteryChargeChange = batteryChargeChangeActions
  
  PyPepper.say( constants.INTRO_TEXT )

  myMessenger = messenger.Messenger()
  if not myMessenger.checkin():
    msgTryTimer = PyPepper.addTimer( 10*60, -1, 10*60 )

