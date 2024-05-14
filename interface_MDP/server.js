
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
            controlRobot(parsedMessage.button);
        } else if (parsedMessage.command === 'schedule') {
            // Call function to handle scheduling
            addScheduledCleaning(parsedMessage.data.date, parsedMessage.data.time);
        } else {
            console.error('Unknown command:', parsedMessage.command);
        }
    });
});

////////////////////////////WEBSOCKET CONNECTION////////////////////////////////////




//////////////////////////////////////ROS NODE INITIALIZATION/////////////////////////
// Import the necessary modules for ROS interaction
const rosnodejs = require('rosnodejs');

// Initialize ROS node
rosnodejs.initNode('/robot_control').then(() => {
    console.log('ROS node initialized');
}).catch((error) => {
    console.error('Error initializing ROS node:', error);
});
//////////////////////////////////////ROS NODE INITIALIZATION/////////////////////////





/////////////////////////////////////MOVEMENT MANUAL CONTROL//////////////////////////////////

// Function to call ROS service for controlling robot movement
async function controlRobot(command) {
    try {
        // Initialize ROS node
        await rosnodejs.initNode('/robot_control');

        // Get ROS node handle
        const nh = rosnodejs.nh;

        // Define the service map to map commands to ROS service names and parameters
        const serviceMap = {
            forward: { 
                left_front: { service: '/mirte/set_left_front_speed', speed: 70 },
                right_front: { service: '/mirte/set_right_front_speed', speed: 70 },
                left_back: { service: '/mirte/set_left_back_speed', speed: 70 },
                right_back: { service: '/mirte/set_right_back_speed', speed: 70 }
            },
            backward: { 
                left_front: { service: '/mirte/set_left_front_speed', speed: -70 },
                right_front: { service: '/mirte/set_right_front_speed', speed: -70 },
                left_back: { service: '/mirte/set_left_back_speed', speed: -70 },
                right_back: { service: '/mirte/set_right_back_speed', speed: -70 }
            },
            left: { 
                right_front: { service: '/mirte/set_right_front_speed', speed: 70 },
                right_back: { service: '/mirte/set_right_back_speed', speed: 70 }
            },
            right: { 
                left_front: { service: '/mirte/set_left_front_speed', speed: 70 },
                left_back: { service: '/mirte/set_left_back_speed', speed: 70 }
            },
            stop: { 
                left_front: { service: '/mirte/set_left_front_speed', speed: 0 },
                right_front: { service: '/mirte/set_right_front_speed', speed: 0 },
                left_back: { service: '/mirte/set_left_back_speed', speed: 0 },
                right_back: { service: '/mirte/set_right_back_speed', speed: 0 }
            }
        };

        // Get the ROS service information for the command
        const services = serviceMap[command];

        if (services) {
            // Call the ROS services with the appropriate parameters
            Object.entries(services).forEach(([wheel, serviceInfo]) => {
                console.log(`Sending message to ${wheel} wheel: ${JSON.stringify({ service: serviceInfo.service, speed: serviceInfo.speed })}`);

                // Create ROS service client for the specified service name
                const serviceClient = nh.serviceClient(serviceInfo.service, 'std_srvs/SetInt');

                // Create request message
                const request = new rosnodejs.ServiceRequest({
                    speed: serviceInfo.speed
                });

                // Call the ROS service
                serviceClient.call(request, (response) => {
                    console.log(`ROS service response for ${wheel} wheel:`, response);
                });
            });
        } else {
            console.log('Unknown command:', command);
        }

        // Shutdown ROS node
        rosnodejs.shutdown();
    } catch (error) {
        console.error('Error in controlRobot function:', error);
    }
}
//////////////////////////////////////MOVEMENT MANUAL CONTROL//////////////////////////////////

////////////////////////////////////SCHEDULE CLEANING/////////////////////////////////////
const { scheduleJob } = require('node-schedule');

function startCleaning() {
    console.log('Starting cleaning process...');
    // Add your cleaning logic here
}

// Define a list to store scheduled dates and times
let scheduledCleanings = [];

// Function to add a scheduled cleaning session
function addScheduledCleaning(date, time) {
    scheduledCleanings.push({ date, time });
}

function checkScheduledCleanings() {
    const currentTime = new Date();
    const currentDateString = currentTime.toISOString().split('T')[0]; // Get current date in format 'YYYY-MM-DD'
    const currentTimeString = currentTime.toTimeString().split(' ')[0]; // Get current time in format 'HH:MM:SS'

    console.log('Checking scheduled cleanings...');
    console.log('Current Date:', currentDateString);
    console.log('Current Time:', currentTimeString);

    scheduledCleanings.forEach(scheduledCleaning => {
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
            startCleaning();
        }
    });
}

// Schedule periodic checking of scheduled cleanings
setInterval(checkScheduledCleanings, 10000); // Check every 10 seconds

////////////////////////////////////SCHEDULE CLEANING/////////////////////////////////////




