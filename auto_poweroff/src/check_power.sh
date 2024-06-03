#!/bin/bash

# Define the path to the file
FILE_PATH="/home/mirte/power_check.txt"

while true; do
    # Wait for 5 minutes
    sleep 300
    
    # Check if the file exists
    if [ -f "$FILE_PATH" ]; then
        # Read the integer from the file
        FILE_TIME=$(cat "$FILE_PATH")
        
        # Get the current time in seconds since epoch
        CURRENT_TIME=$(date +%s)
        
        # Calculate the difference
        TIME_DIFF=$((CURRENT_TIME - FILE_TIME))
        
        # Compare the difference with 300 seconds
        if [ $TIME_DIFF -gt 300 ]; then
            # Execute poweroff if the difference is greater than 300 seconds
            sudo poweroff
        fi
    fi
done