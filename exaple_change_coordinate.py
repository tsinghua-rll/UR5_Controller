import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate controlling robot by keypad
#
# Notice: holding a key will make robot accumulate the step length

def main(ROBOT_HOST = "192.168.1.104"):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI(ROBOT_HOST)

    api.set_coordinate_origin((0.5774, -0.1161, 0.2100))

    api.MoveEndPointToPosition(pos=(0.0, 0.0, 0.0),
                               rotation=(2.2144, -2.2144, 0))
    while not api.isLastMovementEnd():
        time.sleep(0.5)
    time.sleep(2)
    api.FineTuningPosition(interactive=True, add_value=(0.0005, 0.0005, 0.0005, 0.001, 0.001, 0.001), factor=50.0)

if __name__ == "__main__":
    main()
