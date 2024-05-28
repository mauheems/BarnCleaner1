
///////////////////////////////////////SERVER//////////////////////////////////////////
const http = require('http');
const fs = require('fs');
const path = require('path');



const server = http.createServer((req, res) => {
    // Log the URL of the request
    console.log(`Request URL: ${req.url}`);

    // Serve the index.html file
    if (req.url === '/' || req.url === '/index.html') {
        fs.readFile(path.join(__dirname, 'index.html'), (err, data) => {
            if (err) {
                res.writeHead(500, { 'Content-Type': 'text/plain' });
                res.end('Internal Server Error');
                return;
            }
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(data);
        });
    } else {
        // Handle other requests (e.g., script.js)
        const filePath = path.join(__dirname, req.url);
        fs.readFile(filePath, (err, data) => {
            if (err) {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('File Not Found');
                return;
            }
            res.writeHead(200, { 'Content-Type': 'text/javascript' });
            res.end(data);
        });
    }
});

// Start the server
const port = 8080;
server.listen(port, () => {
    console.log(`Server is running on http://localhost:${port}`);
});
///////////////////////////////SERVER//////////////////////////////////


////////////////////////////WEBSOCKET CONNECTION////////////////////////////////////
// WebSocket connection for logging button presses
const WebSocket = require('ws');
const wss = new WebSocket.Server({ server });

// WebSocket connection for handling button presses and scheduling commands
wss.on('connection', ws => {
    console.log('WebSocket connected');
    
    // Send the current list of scheduled cleanings to the new client
    sendScheduledCleanings(ws);
    
    // Handle WebSocket messages
    ws.on('message', message => {
        const messageString = message.toString();
        console.log('Received message:', messageString);

        // Parse the message as JSON
        let parsedMessage;
        try {
            parsedMessage = JSON.parse(messageString);
        } catch (error) {
            console.error('Error parsing message:', error);
            return;
        }

        // Check if it's a button press command
        if (parsedMessage.command === 'buttonPress') {
            console.log('Button pressed:', parsedMessage.button);
            // Control the robot based on the button pressed
            controlRobot(ws, parsedMessage.button);
        } else if (parsedMessage.command === 'schedule') {
            // Call function to handle scheduling
            addScheduledCleaning(parsedMessage.data.date, parsedMessage.data.time);
        } else {
            console.error('Unknown command:', parsedMessage.command);
        }
    });
});

// Establish WebSocket connection with the ROSBridge server
var ws9090 = new WebSocket('ws://localhost:9090');
var ws = new WebSocket('ws://localhost:8080');

// Log a message when the WebSocket connection is opened
ws9090.onopen = function() {
    console.log('WebSocket connected');
    
    // Subscribe to the ROS topic /power/power_watcher
    const powerSubscription = {
        op: 'subscribe',
        id: 'power_subscription',
        topic: '/mirte/power/power_watcher',
        type: 'sensor_msgs/BatteryState',
    };
    
    ws9090.send(JSON.stringify(powerSubscription));

};


// Handle incoming messages from the WebSocket
ws9090.onmessage = function(event) {
    const message = JSON.parse(event.data);
    
    // Check if the message is from the power watcher topic
    if (message.topic === '/mirte/power/power_watcher') {
    
        // Extract the percentage value from the message
        const percentage = message.msg.percentage * 100; // Convert to percentage
        
        
        // Send the percentage value to the client
        ws.send(JSON.stringify({op: 'battery_percentage', percentage}));
    }
};



////////////////////////////WEBSOCKET CONNECTION////////////////////////////////////




//////////////////////////////////////ROS NODE INITIALIZATION/////////////////////////
// Import the necessary modules for ROS interaction
const rosnodejs = require('rosnodejs');

// Initialize ROS node and handle
rosnodejs.initNode('/robot_control')
    .then((rosNode) => {
        const nh = rosNode;

        console.log('ROS node initialized');
    })
    .catch((error) => {
        console.error('Error initializing ROS node:', error);
    });

//////////////////////////////////////ROS NODE INITIALIZATION/////////////////////////




/////////////////////////////////////MOVEMENT MANUAL CONTROL//////////////////////////////////



// Function to send control commands via WebSocket
function controlRobot(ws, command) {
    // Define the service map to map commands to ROS service names and parameters
    const serviceMap = {
        forward: { 
            left_front: { service: '/mirte/set_left_front_speed', speed: 70 },
            right_front: { service: '/mirte/set_right_front_speed', speed: 70 },
            left_back: { service: '/mirte/set_left_rear_speed', speed: 70 },
            right_back: { service: '/mirte/set_right_rear_speed', speed: 70 }
        },
        backward: { 
            left_front: { service: '/mirte/set_left_front_speed', speed: -70 },
            right_front: { service: '/mirte/set_right_front_speed', speed: -70 },
            left_back: { service: '/mirte/set_left_rear_speed', speed: -70 },
            right_back: { service: '/mirte/set_right_rear_speed', speed: -70 }
        },
        left: { 
            right_front: { service: '/mirte/set_right_front_speed', speed: 70 },
            right_back: { service: '/mirte/set_right_rear_speed', speed: 70 }
        },
        right: { 
            left_front: { service: '/mirte/set_left_front_speed', speed: 70 },
            left_back: { service: '/mirte/set_left_rear_speed', speed: 70 }
        },
        stop: { 
            left_front: { service: '/mirte/set_left_front_speed', speed: 0 },
            right_front: { service: '/mirte/set_right_front_speed', speed: 0 },
            left_back: { service: '/mirte/set_left_rear_speed', speed: 0 },
            right_back: { service: '/mirte/set_right_rear_speed', speed: 0 }
        }
    };

    // Get the ROS service information for the command
    const services = serviceMap[command];

    if (services) {
        // Construct and send WebSocket messages for each service
        Object.entries(services).forEach(([wheel, serviceInfo]) => {
            // Construct the message with the "op" field
            const message = {
                op: 'call_service', // Set the operation to "call_service"
                service: serviceInfo.service,
                type: '/mirte_msgs/SetMotorSpeed',
                args: [{ speed: serviceInfo.speed }]
            };

            // Log the message for debugging
            console.log(`Sending message to ${wheel} wheel: ${JSON.stringify(message)}`);

            // Send the message via WebSocket
            ws9090.send(JSON.stringify(message));
        });
    } else {
        console.log('Unknown command:', command);
    }
}


////////////////MOVEMENT MANUAL CONTROL//////////////////////////////////

////////////////////////////////////SCHEDULE CLEANING/////////////////////////////////////
const { scheduleJob } = require('node-schedule');

// Define a list to store scheduled dates and times
let scheduledCleanings = [];


// Function to start cleaning
function startCleaning(scheduledCleaningIndex) {
    console.log('Starting cleaning for:', scheduledCleanings[scheduledCleaningIndex]);
    // Define the ROS service call details
    const startCleaningCall = {
      op: 'call_service',
      service: '/start_cleaning_service', // Replace with your actual service name
      args: {} // Replace with actual service arguments if needed
    };

    // Log the service call message
    console.log('Service call message:', startCleaningCall);

    // Send the service call message to the WebSocket
    ws9090.send(JSON.stringify(startCleaningCall));
	
    // Remove the scheduled cleaning from the list
    scheduledCleanings.splice(scheduledCleaningIndex, 1);
    
    // Broadcast the updated list of scheduled cleanings to WebSocket clients
    broadcastScheduledCleanings();
    

}

// Function to send the current list of scheduled cleanings to a WebSocket client
function sendScheduledCleanings(ws) {
    const message = JSON.stringify({ op: 'update_scheduled_cleanings', scheduledCleanings });
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(message);
    }
}

function addScheduledCleaning(date, time) {
    scheduledCleanings.push({ date, time });

    // Sort the scheduled cleanings by date and time
    scheduledCleanings.sort((a, b) => {
        const dateA = new Date(`${a.date}T${a.time}`);
        const dateB = new Date(`${b.date}T${b.time}`);
        return dateA - dateB;
    });

    // Send the updated list of scheduled cleanings to WebSocket clients
    broadcastScheduledCleanings();
}

// Function to broadcast the list of scheduled cleanings to WebSocket clients
function broadcastScheduledCleanings() {
    const message = JSON.stringify({ op: 'update_scheduled_cleanings', scheduledCleanings });
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}



// Function to check scheduled cleanings
function checkScheduledCleanings() {
    const currentTime = new Date();
    const currentDateString = currentTime.toISOString().split('T')[0]; // Get current date in format 'YYYY-MM-DD'
    const currentTimeString = currentTime.toTimeString().split(' ')[0]; // Get current time in format 'HH:MM:SS'

    console.log('Checking scheduled cleanings...');
    console.log('Current Date:', currentDateString);
    console.log('Current Time:', currentTimeString);

    scheduledCleanings.forEach((scheduledCleaning, index) => {
        console.log('Scheduled Date:', scheduledCleaning.date);
        console.log('Scheduled Time:', scheduledCleaning.time);

        // Extract hours and minutes from current time and scheduled time
        const [currentHours, currentMinutes] = currentTimeString.split(':');
        const [scheduledHours, scheduledMinutes] = scheduledCleaning.time.split(':');

        // Check if hours and minutes match
        if (
            scheduledCleaning.date === currentDateString &&
            currentHours === scheduledHours &&
            currentMinutes === scheduledMinutes
        ) {
            console.log('Scheduled cleaning matched. Starting cleaning...');
            // Match found, trigger cleaning
            startCleaning(index); // Pass the index of the matched cleaning
        }
    });
}

// Schedule periodic checking of scheduled cleanings
setInterval(checkScheduledCleanings, 10000); // Check every 10 seconds



////////////////////////////////////SCHEDULE CLEANING/////////////////////////////////////




