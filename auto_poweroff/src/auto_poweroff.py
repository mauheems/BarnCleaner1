from sensor_msgs.msg import BatteryState
import rospy
import subprocess
import time


class AutoPoweroff:
    def __init__(self) -> None:
        """
        Initializes the AutoPoweroff class.

        This method sets the initial values for the battery state, subscribes to the
        "/mirte/power/power_watcher" topic to receive battery state updates, sets the
        write path for the power check file, and sets the last push time to the current
        time minus 300 seconds.

        Parameters:
            None

        Returns:
            None
        """
        self.battery_state: float = 0.0
        self.sub_battery = rospy.Subscriber(
            "/mirte/power/power_watcher", BatteryState, self.battery_callback
        )
        self.write_path = "/home/mirte/power_check.txt"
        self.last_push = time.time() - 300

    def battery_callback(self, data: BatteryState):
        """
        Callback function for battery state updates.

        Args:
            data (BatteryState): The battery state data received.

        Returns:
            None

        This function is called whenever a new battery state update is received. It checks the battery percentage
        and performs the following actions based on the percentage:

        - If the battery percentage is greater than 0.30:
            - Writes the current timestamp to a file specified by `self.write_path`.
            - If the last push was more than 300 seconds ago, sends a message to the wall with the current power level.
            - Updates `self.last_push` to the current time.
        - If the battery percentage is greater than 0.20:
            - Writes the current timestamp to a file specified by `self.write_path`.
            - Sends a message to the wall warning that the robot will turn off soon with the current power level.
        - If the battery percentage is less than or equal to 0.20:
            - Logs an error message indicating that the power is too low and auto poweroff is initiated.
            - Runs the `sudo poweroff` command to turn off the robot.

        Note: This function does not return any value.
        """
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
