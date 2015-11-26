import PyPepper
import constants

status = constants.CLEAN_SLATE

def updateStatus( state, negate ):
  global status
  #TOD should check state first
  if negate:
    status = status &  ~state
  else:
    status = status | state

  #set chest LED
  if status & constants.NEW_MESSAGES:
    PyPepper.pluseChestLED( 'red', 'white', 0.5 )
  elif status & constants.ARCHIVE_MESSAGES:
    if status & constants.USER_PRESENT:
      PyPepper.setChestLED( 'pink' )
    else:
      PyPepper.setChestLED( 'red' )
  elif status & constants.USER_PRESENT:
    PyPepper.setChestLED( 'green' )
  else:
    PyPepper.setChestLED( 'white' )
