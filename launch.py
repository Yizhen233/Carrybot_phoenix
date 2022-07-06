import roslaunch
import time

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [""])

signal = 0

if signal == 1:
    tracking_launch.shutdown()
    move(3)
    Stop = True
else:
    if signal == 0:
        tracking_launch.start()
        print ('starttotrace')
        starttotrace()
        time.sleep(20)
        signal = 1
    else:
        pass
