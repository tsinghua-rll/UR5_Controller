import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate controlling robot by keypad
#
# Notice: holding a key will make robot accumulate the step length


def main():
    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI("192.168.1.104")

    api.set_coordinate_origin((0.46945, -0.00105, 0.04308))

    api.MoveEndPointToPosition(pos=(0.0, 0.0, 0.0),
                               rotation=(2.2144, -2.2144, 0))
    while not api.isLastMovementEnd():
        time.sleep(0.5)
    time.sleep(2)
    api.FineTuningPosition(interactive=True, add_value=(0.0005, 0.0005, 0.0005, 0.001, 0.001, 0.001), factor=10.0)

if __name__ == "__main__":
    main()
