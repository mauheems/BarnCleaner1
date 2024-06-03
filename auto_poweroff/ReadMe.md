# Battery monitor and auto poweroff

- Script to run the auto_poweroff_node that monitors battery percentage.
- Every 5 minutes, it sends a [wall](https://en.wikipedia.org/wiki/Wall_(Unix)) message informing user about the battery percentage.
- If battery is below 30%, it will warn the user to charge the robot.
- If the battery is below 20%, it will turn of the robot.
