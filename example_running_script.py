import logging
import time
from RTIF.API import API

# this script demonstrate the usage of 'get' functions


def main(ROBOT_HOST = "192.168.1.104"):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = API(ROBOT_HOST)

    scripy_path = "./hammer.script"
    print ("Now start running script")
    api.run_script(scripy_path)
    # sleep for 5 min to keep script running
    if keep_running:
        time.sleep(300)


if __name__ == "__main__":
    main()
