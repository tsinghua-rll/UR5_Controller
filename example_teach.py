import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate the teach mode of robot. When running this script,
# the robot arm can be move by human hand,
# press enter to capture a key point, and e+enter to start replay
#
# Notice: teach mode will automatically terminated after 3 minutes by robot
# so when you cannot move the robot arm, use e+enter to exit teach mode

def main(ROBOT_HOST = "192.168.1.104"):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI(ROBOT_HOST)

    print ("Lets start teaching")
    caps = api.RecordingActions()

    # control loop
    while keep_running:
        print ("Now, lets replay the record")
        api.ReplayCapturedData(caps, with_time=True)
        time.sleep(3)

if __name__ == "__main__":
    main()

