from sensor_msgs.msg import BatteryState
import rospy
import subprocess
import time

class AutoPoweroff:
    def __init__(self) -> None:
        self.battery_state: float = 0.0
        self.sub_battery = rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)
        self.write_path = '/home/mirte/power_check.txt'

    def battery_callback(self, data: BatteryState):
        ti = str(int(time.time()))
        if data.percentage > 0.20:
            with open(self.write_path, 'w') as f:
                f.write(ti)
                f.close()
        else:
            rospy.logerr("Power too low. Auto poweroff now")
            subprocess.run(['sudo', 'poweroff', 'now'], check=True)
        return


