// Establish WebSocket connection with the server
var ws = new WebSocket('ws://localhost:8080');

// Log a message when the WebSocket connection is opened
ws.onopen = function() {
    console.log('WebSocket connected');
};

//////////////////////////////////Manual control//////////////////////////////////////
// Variable to track if the button is pressed
var buttonPressed = false;

// Function to send button press information to the server
function sendButtonPress(command) {
    const manualBtn = document.getElementById('manualBtn');
    
    // Check if manual control is active or if the button is not pressed
    if (manualBtn.classList.contains('active') || !buttonPressed) {
        // Send JSON-formatted message with 'op' field
        ws.send(JSON.stringify({ op: 'call_service', service: '/mirte/set_left_front_speed', speed: 70, command: 'buttonPress', button: command }));
    }
}

// Function to handle mouse down event
function handleMouseDown(command) {
    buttonPressed = true;
    sendButtonPress(command);
}

// Function to handle mouse up event
function handleMouseUp() {
    // Check if the button was pressed
    if (buttonPressed) {
        // Reset the flag indicating the button is pressed
        buttonPressed = false;
        // Send a command to stop the robot (set speed to 0)
        sendButtonPress('stop');
    }
}

// Add event listeners to the arrow buttons
document.getElementById('forwardBtn').addEventListener('mousedown', function() {
    handleMouseDown('forward');
});
document.getElementById('backwardBtn').addEventListener('mousedown', function() {
    handleMouseDown('backward');
});
document.getElementById('leftBtn').addEventListener('mousedown', function() {
    handleMouseDown('left');
});
document.getElementById('rightBtn').addEventListener('mousedown', function() {
    handleMouseDown('right');
});

// Add mouseup event listener to the document
document.addEventListener('mouseup', handleMouseUp);

// Add mouseup event listener to the document
document.addEventListener('mouseup', handleMouseUp);
document.addEventListener("DOMContentLoaded", function() {
    // Manual control button
    const manualBtn = document.getElementById('manualBtn');

    // Manual control buttons
    const forwardBtn = document.getElementById('forwardBtn');
    const leftBtn = document.getElementById('leftBtn');
    const rightBtn = document.getElementById('rightBtn');
    const backwardBtn = document.getElementById('backwardBtn');

    // Event listener for manual control button
    manualBtn.addEventListener('click', function() {
        // Toggle the enabled state of the joystick icon and the arrow buttons
        const isManual = manualBtn.classList.contains('active');
        if (!isManual) {
            // Enable manual control
            manualBtn.classList.add('active');
            forwardBtn.disabled = false;
            leftBtn.disabled = false;
            rightBtn.disabled = false;
            backwardBtn.disabled = false;
        } else {
            // Disable manual control
            manualBtn.classList.remove('active');
            forwardBtn.disabled = true;
            leftBtn.disabled = true;
            rightBtn.disabled = true;
            backwardBtn.disabled = true;
        }
    });

    // Event listener for the arrow buttons
    const arrowButtons = [forwardBtn, leftBtn, rightBtn, backwardBtn];
    arrowButtons.forEach(button => {
        button.addEventListener('click', function() {
            const isManual = manualBtn.classList.contains('active');
            if (isManual) {
                sendButtonPress(button.id.replace('Btn', ''));
            }
        });
    });
});


//////////////////////////////////Manual control//////////////////////////////////////



/////////////////////////////////////NOTIFICATIONS//////////////////////////////////
// Function to display a notification in the banner
function showNotification(message) {
    var notificationBanner = document.getElementById('notification-banner');
    notificationBanner.textContent = message;
    notificationBanner.style.display = 'block';

    // Hide the notification after a certain time (e.g., 5 seconds)
    setTimeout(function() {
        notificationBanner.style.display = 'none';
    }, 5000); // 5000 milliseconds = 5 seconds
}

// Example usage: 
showNotification('Robot is connected!');

/////////////////////////////////////NOTIFICATIONS//////////////////////////////////



/////////////////////////////////////BATTERY///////////////////////////////////////
// WebSocket connection for receiving power information
ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    if (message.op === 'publish' && message.topic === '/power/power_watcher') {
        // Update battery parameter in HTML
        document.getElementById('batteryParameter').innerText = `Battery: ${message.msg.data.toFixed(2)}%`;
        console.log(`Battery percentage: ${message.msg.data.toFixed(2)}%`);
    }
};
/////////////////////////////////////BATTERY///////////////////////////////////////


///////////////////////////////////STATUS//////////////////////////////////

// Function to update the status of each robot
function updateRobotStatus(robotId, status) {
    var robotStatusElement = document.getElementById(`status${robotId}`);
    robotStatusElement.innerHTML = `Status: ${status} ${getStatusIcon(status)}`;
    addStatusUpdate(robotId, status);
}

// Example usage:
updateRobotStatus(1, 'cleaning');
updateRobotStatus(2, 'charging');
updateRobotStatus(3, 'stuck');
updateRobotStatus(1, 'cleaning');
updateRobotStatus(2, 'charging');
updateRobotStatus(3, 'stuck');
updateRobotStatus(1, 'cleaning');
updateRobotStatus(2, 'charging');
updateRobotStatus(3, 'stuck');

// Function to get the status icon based on the status text
function getStatusIcon(status) {
    var iconClass = '';
    if (status.includes('cleaning')) {
        iconClass = 'fa-broom'; // Icon for cleaning
    } else if (status.includes('charging')) {
        iconClass = 'fa-charging-station'; // Icon for charging
    } else if (status.includes('stuck')) {
        iconClass = 'fa-exclamation-triangle'; // Icon for stuck
    }
    return `<i class="fas ${iconClass}"></i>`;
}


// Function to add a status update to the log with timestamp, robot ID, and status icon
function addStatusUpdate(robotId, status) {
    var statusList = document.getElementById('status-list');
    var listItem = document.createElement('li');

    // Get the current time
    var currentTime = new Date();
    var timestamp = currentTime.toLocaleString(); // Format timestamp as a string

    // Create a string with status, timestamp, and robot ID
    var statusWithTimestamp = `${timestamp}: Robot ${robotId} - ${status}`;

    // Append the status text and icon to the list item
    listItem.innerHTML = `${statusWithTimestamp} ${getStatusIcon(status)}`;

    // Add CSS classes for styling
    listItem.classList.add('status-item');

    // Add the status update to the top of the log
    if (statusList.firstChild) {
        statusList.insertBefore(listItem, statusList.firstChild);
    } else {
        statusList.appendChild(listItem);
    }
}


// JavaScript file
document.addEventListener("DOMContentLoaded", function() {
    // Add event listener to the clear status log button
    const clearStatusLogBtn = document.getElementById('clearStatusLogBtn');
    if (clearStatusLogBtn) {
        clearStatusLogBtn.addEventListener('click', clearStatusLog);
    }
});

// Function to clear the status log
function clearStatusLog() {
    const statusList = document.getElementById('status-list');
    if (statusList) {
        // Clear all status log entries
        statusList.innerHTML = '';
    }
}

//////////////////////////////////STATUS///////////////////////////////////////////

/////////////////////////////////SCHEDULE CLEANING///////////////////////////////////
// WebSocket connection for handling scheduled cleaning updates
ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    if (message.op === 'update_scheduled_cleanings') {
        const scheduledCleanings = message.scheduledCleanings;
        updateScheduledCleaningsList(scheduledCleanings);
    }
};

// Function to update the list of scheduled cleanings in the HTML
function updateScheduledCleaningsList(scheduledCleanings) {
    var scheduledCleaningsList = document.getElementById('scheduledCleaningsList');
    scheduledCleaningsList.innerHTML = ''; // Clear the existing list

    // Iterate over the scheduled cleanings and create list items for each
    scheduledCleanings.forEach((scheduledCleaning, index) => {
        var listItem = document.createElement('li');
        listItem.textContent = `Date: ${scheduledCleaning.date}, Time: ${scheduledCleaning.time}`;
        scheduledCleaningsList.appendChild(listItem);
    });
}


// Function to handle form submission for scheduling cleaning session
function handleFormSubmission(event) {
    event.preventDefault(); // Prevent the default form submission behavior

    // Get the form data
    var formData = new FormData(event.target);
    var scheduleDate = formData.get('scheduleDate');
    var scheduleTime = formData.get('scheduleTime');

    // Create a schedule object
    var schedule = {
        date: scheduleDate,
        time: scheduleTime
    };
    
    // Send the schedule data to the server via WebSocket
    ws.send(JSON.stringify({ command: 'schedule', data: schedule }));

    // Reset the form
    event.target.reset();
}

// Add an event listener to the form for submission
document.getElementById('scheduleForm').addEventListener('submit', handleFormSubmission);

//////////////////////////////SCHEDULE CLEANING////////////////////////////////////////

