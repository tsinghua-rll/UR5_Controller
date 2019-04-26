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
    api.MoveEndPointToPosition(pos=(0.5774472131510267, -0.11612032542570545, 0.21002231707422814),
                               rotation=(2.2558090087495724, -2.1853634973877805, 0.016568855033798274))
    while not api.isLastMovementEnd():
        time.sleep(0.5)
    time.sleep(2)
    api.FineTuningPosition(interactive=True, add_value=(0.0005, 0.0005, 0.0005, 0.0001, 0.0001, 0.0001), factor=50.0)

if __name__ == "__main__":
    main()
