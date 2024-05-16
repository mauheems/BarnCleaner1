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
    var manualBtn = document.getElementById('manualBtn');
    if (manualBtn.textContent === 'Automatic Control' || buttonPressed === false) {
        // Send JSON-formatted message
        ws.send(JSON.stringify({ command: 'buttonPress', button: command }));
    }
}

// Function to handle mouse down event
function handleMouseDown(command) {
    buttonPressed = true;
    sendButtonPress(command); // Change this to the appropriate command
}

// Function to handle mouse up event
function handleMouseUp() {
    buttonPressed = false;
    // Send a command to stop the robot (set speed to 0)
    ws.send(JSON.stringify({ command: 'buttonPress', button: 'stop' })); // Send JSON-formatted message for button release
}


// Add event listeners to the buttons
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


// Event listener for the manual control button
document.getElementById('manualBtn').addEventListener('click', function() {
    var manualBtn = document.getElementById('manualBtn');
    var forwardBtn = document.getElementById('forwardBtn');
    var backwardBtn = document.getElementById('backwardBtn');
    var leftBtn = document.getElementById('leftBtn');
    var rightBtn = document.getElementById('rightBtn');

    // Toggle the enabled state of directional buttons
    if (manualBtn.textContent === 'Manual Control') {
        forwardBtn.disabled = false;
        backwardBtn.disabled = false;
        leftBtn.disabled = false;
        rightBtn.disabled = false;
        manualBtn.textContent = 'Automatic Control';
        console.log('Manual Control');
    } else {
        forwardBtn.disabled = true;
        backwardBtn.disabled = true;
        leftBtn.disabled = true;
        rightBtn.disabled = true;
        manualBtn.textContent = 'Manual Control';
        console.log('Automatic Control');
    }
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



///////////////////////////////////STATUS//////////////////////////////////

// Function to add a status update to the log with timestamp
function addStatusUpdate(status) {
    var statusList = document.getElementById('status-list');
    var listItem = document.createElement('li');
    
    // Get the current time
    var currentTime = new Date();
    var timestamp = currentTime.toLocaleString(); // Format timestamp as a string
    
    // Create a string with status and timestamp
    var statusWithTimestamp = `${timestamp}: ${status}`;
    listItem.textContent = statusWithTimestamp;
    
    // Add the status update to the log
    statusList.appendChild(listItem);
}

// Function to clear the status log
function clearStatusLog() {
    var statusList = document.getElementById('status-list');
    statusList.innerHTML = ''; // Clear all status updates
}

// Example usage:
addStatusUpdate('Robot is charging');
//////////////////////////////////STATUS///////////////////////////////////////////

/////////////////////////////////SCHEDULE CLEANING///////////////////////////////////


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

