from sensor_msgs.msg import BatteryState
import rospy
import subprocess
import time


class AutoPoweroff:
    def __init__(self) -> None:
        self.battery_state: float = 0.0
        self.sub_battery = rospy.Subscriber(
            "/mirte/power/power_watcher", BatteryState, self.battery_callback
        )
        self.write_path = "/home/mirte/power_check.txt"
        self.last_push = time.time() - 300

    def battery_callback(self, data: BatteryState):
        ti = str(int(time.time()))
        power_level = str(data.percentage * 100)
        if data.percentage > 0.30:
            with open(self.write_path, "w") as f:
                f.write(ti)
                f.close()
            if time.time() - self.last_push > 300:
                subprocess.run(["wall", "Power level at " + power_level])
                self.last_push = time.time()
        elif data.percentage > 0.20:
            with open(self.write_path, "w") as f:
                f.write(ti)
                f.close()
            subprocess.run(
                ["wall", "The robot will turn off soon. Power level at " + power_level]
            )
        else:
            rospy.logerr("Power too low. Auto poweroff now")
            subprocess.run(["sudo", "poweroff"], check=True)
        return
