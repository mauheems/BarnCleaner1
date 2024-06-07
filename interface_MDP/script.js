// Establish WebSocket connection with the server
var ws = new WebSocket('ws://localhost:8080');

// Log a message when the WebSocket connection is opened
ws.onopen = function() {
    console.log('WebSocket connected');
    showNotification('Robot is connected!');
};

//////////////////////////////////Manual control//////////////////////////////////////

// Variable to track if the button is pressed
var buttonPressed = false;

// Function to send button press information to the server
function sendButtonPress(command) {
    const manualBtn = document.getElementById('manualBtn');
    
    // Check if manual control is active
    if (manualBtn.classList.contains('active')) {
        // Send JSON-formatted message with 'op' field
        ws.send(JSON.stringify({ op: 'call_service', command: 'buttonPress', button: command }));
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

// Function to handle key down event
function handleKeyDown(event) {
    if (event.repeat) return;  // Ignore repeated keydown events
    switch (event.key) {
        case 'ArrowUp':
            handleMouseDown('forward');
            break;
        case 'ArrowDown':
            handleMouseDown('backward');
            break;
        case 'ArrowLeft':
            handleMouseDown('left');
            break;
        case 'ArrowRight':
            handleMouseDown('right');
            break;
    }
}

// Function to handle key up event
function handleKeyUp(event) {
    switch (event.key) {
        case 'ArrowUp':
        case 'ArrowDown':
        case 'ArrowLeft':
        case 'ArrowRight':
            handleMouseUp();
            break;
    }
}

// Add event listeners to the arrow buttons
document.getElementById('forwardBtn').addEventListener('mousedown', function() {
    handleMouseDown('forward');
});
document.getElementById('forwardBtn').addEventListener('mouseup', handleMouseUp);

document.getElementById('backwardBtn').addEventListener('mousedown', function() {
    handleMouseDown('backward');
});
document.getElementById('backwardBtn').addEventListener('mouseup', handleMouseUp);

document.getElementById('leftBtn').addEventListener('mousedown', function() {
    handleMouseDown('left');
});
document.getElementById('leftBtn').addEventListener('mouseup', handleMouseUp);

document.getElementById('rightBtn').addEventListener('mousedown', function() {
    handleMouseDown('right');
});
document.getElementById('rightBtn').addEventListener('mouseup', handleMouseUp);

// Add mouseup event listener to the document to handle mouse up outside of buttons
document.addEventListener('mouseup', handleMouseUp);

// Add keyboard event listeners to the document
document.addEventListener('keydown', handleKeyDown);
document.addEventListener('keyup', handleKeyUp);

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
            updateRobotStatus(1, 'manual');
            // Enable manual control
            manualBtn.classList.add('active');
            forwardBtn.disabled = false;
            leftBtn.disabled = false;
            rightBtn.disabled = false;
            backwardBtn.disabled = false;
        } else {
            // Disable manual control
            updateRobotStatus(1, 'waiting for commands');
            manualBtn.classList.remove('active');
            forwardBtn.disabled = true;
            leftBtn.disabled = true;
            rightBtn.disabled = true;
            backwardBtn.disabled = true;
        }
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

/////////////////////////////////////NOTIFICATIONS//////////////////////////////////


///////////////////////////////////STATUS//////////////////////////////////

// Function to update the status of each robot
function updateRobotStatus(robotId, status) {
    var robotStatusElement = document.getElementById(`status${robotId}`);
    robotStatusElement.innerHTML = `Status: ${status} ${getStatusIcon(status)}`;
    addStatusUpdate(robotId, status);
}

// Function to get the status icon based on the status text
function getStatusIcon(status) {
    var iconClass = '';
    if (status.includes('cleaning')) {
        iconClass = 'fa-broom'; // Icon for cleaning
    } else if (status.includes('charging')) {
        iconClass = 'fa-charging-station'; // Icon for charging
    } else if (status.includes('stuck')) {
        iconClass = 'fa-exclamation-triangle'; // Icon for stuck
    } else if (status.includes('mapping')) {
        iconClass = 'fa-map'; // Icon for mapping
    } else if (status.includes('manual')) {
        iconClass = 'fas fa-gamepad'; // Icon for manual control
    } else if (status.includes('waiting for commands')) {
        iconClass = 'fa-hourglass-half'; // Icon for waiting
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

// Event listener for the "Yep" button in the modal to clear the status log
document.getElementById('confirmClearStatusLog').addEventListener('click', function() {
    // Clear the status log
    document.getElementById('status-list').innerHTML = '';
    // Hide the modal
    $('#clearStatusModal').modal('hide');
});

// Initial status update
updateRobotStatus(1, 'waiting for commands');
updateRobotStatus(2, 'waiting for commands');
updateRobotStatus(3, 'waiting for commands');
///////////////////////////////////STATUS//////////////////////////////////



/////////////////////////////////SCHEDULE CLEANING///////////////////////////////////
// WebSocket connection for handling scheduled cleaning updates
ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    
    if (message.op === 'update_scheduled_cleanings') {
        const scheduledCleanings = message.scheduledCleanings;
        updateScheduledCleaningsList(scheduledCleanings);
    }
    // Handle battery percentage updates
    else if (message.op === 'battery_percentage') {
        const percentage = message.percentage;
        document.getElementById('batteryPercentage').textContent = percentage + '%';
    }
    
    else if (message.topic === '/map') {
        drawMap(message.msg);
    }
    
    else if (message.op === 'waypoints') {
        updateProgressBar();
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

// Event listener for the "Yes, Clear" button in the modal to clear the scheduled cleanings log
document.getElementById('confirmClearScheduledCleanings').addEventListener('click', function() {
    // Clear the scheduled cleanings log
    document.getElementById('scheduledCleaningsList').innerHTML = '';
    // Hide the modal
    $('#clearScheduledCleaningsModal').modal('hide');
});

document.getElementById('startCleaningButton').addEventListener('click', () => {
        ws.send(JSON.stringify({ command: 'startCleaning'}));
        updateRobotStatus(1, 'cleaning');
        updateRobotStatus(2, 'cleaning');
        updateRobotStatus(3, 'cleaning');
});

document.getElementById('stopCleaningButton').addEventListener('click', () => {
        ws.send(JSON.stringify({ command: 'stopCleaning'}));
        updateRobotStatus(1, 'manually stopped cleaning and waiting for commands');
        updateRobotStatus(2, 'manually stopped cleaning and waiting for commands');
        updateRobotStatus(3, 'manually stopped cleaning and waiting for commands');
});

updateProgressBar();

// Function to handle the cleaning progress
function updateProgressBar() {
    var totalWaypoints = 10;
    var waypointsDone = 5;
    if (totalWaypoints > 0) {
        const progressPercent = (waypointsDone / totalWaypoints) * 100;
        const progressBar = document.getElementById('cleaningProgressBar');
        const progressValue = document.getElementById('progressValue');
        
        progressBar.style.width = progressPercent + '%';
        progressBar.setAttribute('aria-valuenow', progressPercent);
        progressValue.textContent = `${waypointsDone}/${totalWaypoints}`;
    }
}
/////////////////////////////////SCHEDULE CLEANING///////////////////////////////////



/////////////////////////////////////////MAPPING////////////////////////////////////////////////


document.getElementById('confirmStartMapping').onclick = () => {
    ws.send(JSON.stringify({ command: 'startMapping' }));
    $('#startMappingModal').modal('hide');
    updateRobotStatus(1, 'mapping');
};


function drawMap(mapData) {
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    const width = mapData.info.width;
    const height = mapData.info.height;
    const data = mapData.data;

    const imgData = ctx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
        const value = data[i];
        const color = value === -1 ? 128 : value === 100 ? 0 : 255;
        const index = i * 4;
        imgData.data[index] = color;
        imgData.data[index + 1] = color;
        imgData.data[index + 2] = color;
        imgData.data[index + 3] = 255; // alpha channel
    }

    ctx.putImageData(imgData, 0, 0);
}

document.getElementById('confirmSaveMap').addEventListener('click', function() {
    ws.send(JSON.stringify({ command: 'saveMap' }));
    // Replace with your save map functionality
    console.log('Map save confirmed');
    
    // Close the modal
    $('#saveMapModal').modal('hide');
});


// Add event listener to the stop mapping button
document.getElementById('confirmStopMappingBtn').addEventListener('click', function () {
    ws.send(JSON.stringify({ command: 'stopMapping' }));
    // Replace with your save map functionality
    console.log('Map save confirmed');
    $('#stopMappingModal').modal('hide');
});
/////////////////////////////////////////MAPPING/////////////////////////////////////////////////



