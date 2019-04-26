import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate the usage of force control
#
# Notice: when controlling the force along Z axis, we should shake the arm
# for an initial acceleration, or the robot will not move.


def main(ROBOT_HOST = "192.168.1.104"):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI(ROBOT_HOST)

    api.MoveEndPointToPosition(pos=(0.5774472131510267, -0.11612032542570545, 0.21002231707422814),
                               rotation=(2.2558090087495724, -2.1853634973877805, 0.016568855033798274))

    while not api.isLastMovementEnd():
        time.sleep(0.5)
    time.sleep(2)
    print ("Start force mode, hand will give a 10N force toward ground")
    api.switch_mode(direct_mode=False)
    for _ in range(3):
        api.ForceMode(selection=(0, 0, 1, 0, 0, 0), wrench=(0, 0, -10.0, 0, 0, 0), limits=(0.5, 0.5, 20.0, 0.5, 0.5, 0.5), duration=0)
        api.MoveEndPointToPosition(pos=(0.6774472131510267, -0.11612032542570545, 0.21002231707422814), v=0.05)
        api.MoveEndPointToPosition(pos=(0.4774472131510267, -0.11612032542570545, 0.21002231707422814), v=0.05)
        api.Sleep(5.0)
        api.run_buffer()
        print ("1 Loop End")
        time.sleep(5)

    print ("End force mode")
    api.switch_mode(direct_mode=True)
    api.EndForceMode()

if __name__ == "__main__":
    main()
