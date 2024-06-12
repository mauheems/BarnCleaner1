///////////////////////////////////////SERVER//////////////////////////////////////////
const http = require('http');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');
const ROSLIB = require('roslib');
const express = require('express');
const app = express();

// Initialize the WebSocket connection
const ws = new WebSocket('ws://localhost:8080');

// Create the HTTP server
const server = http.createServer((req, res) => {
    console.log(`Request URL: ${req.url}`);
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

app.use('/pictures', express.static('/pictures/robot1.png', {
    setHeaders: (res) => {
        res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
    }
}));

// Start the server
const port = 8080;
server.listen(port, () => {
    console.log(`Server is running on http://localhost:${port}`);
});

//////////////////////////////ROSLIB/////////////////////////////////////////////////
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090' 
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
  console.log('Connection to websocket server closed.');
});
//////////////////////////////ROSLIB///////////////////////////////////////////


////////////////////////////WEBSOCKET CONNECTION////////////////////////////////////
const wss = new WebSocket.Server({ server });

wss.on('connection', ws => {
    console.log('WebSocket connected');
    
    // Send the current list of scheduled cleanings to the new client
    sendScheduledCleanings(ws);

    ws.on('message', message => {
        const messageString = message.toString();
        console.log('Received message:', messageString);

        let parsedMessage;
        try {
            parsedMessage = JSON.parse(messageString);
        } catch (error) {
            console.error('Error parsing message:', error);
            return;
        }

        if (parsedMessage.command === 'buttonPress') {
            console.log('Button pressed:', parsedMessage.button);
            controlRobot(parsedMessage.button);
        } else if (parsedMessage.command === 'schedule') {
            addScheduledCleaning(parsedMessage.data.date, parsedMessage.data.time);
        } else if (parsedMessage.command === 'startMapping') {
            startMapping();	
        } else if (parsedMessage.command === 'stopMapping') {
            stopMapping();
    	}  else if (parsedMessage.command === 'startCleaning') {
    		startCleaning();  		
    	}  else if (parsedMessage.command === 'stopCleaning') {
    		stopCleaning();    
    	}
    });

    
    
    // Subscribe to the /mirte/power/power_watcher topic
    const batteryStateTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mirte/power/power_watcher',
      messageType: 'sensor_msgs/BatteryState'
    });

    batteryStateTopic.subscribe(message => {
      //console.log('Received message on ' + batteryStateTopic.name + ': ', message);
      ws.send(JSON.stringify({
        op: 'battery_percentage',
        percentage: (message.percentage * 100).toFixed(0)
      }));
    });
});

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


// Function to set motor speed
function setMotorSpeed(ros, serviceName, speed) {
    var setMotorSpeedClient = new ROSLIB.Service({
        ros: ros,
        name: serviceName,
        serviceType: '/mirte_msgs/SetMotorSpeed'
    });

    var request = new ROSLIB.ServiceRequest({
        speed: speed
    });

    setMotorSpeedClient.callService(request, function(result) {
        console.log('Result for service call on ' + setMotorSpeed.name + ': ' + result);
    });
}

// Function to control the robot
function controlRobot(command) {
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

    const services = serviceMap[command];

   
    Object.entries(services).forEach(([wheel, serviceInfo]) => {
        setMotorSpeed(ros, serviceInfo.service, serviceInfo.speed);
        console.log(`Sending speed ${serviceInfo.speed} to ${wheel} wheel using service ${serviceInfo.service}`);
    });
}

////////////////MOVEMENT MANUAL CONTROL//////////////////////////////////

////////////////////////////////////CLEANING/////////////////////////////////////
const { scheduleJob } = require('node-schedule');
let scheduledCleanings = [];

var totalWaypoints = 10;
var waypointsDone = 5;
var isCleaning = false;

const waypointsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/global_mission/waypoints',
    messageType: 'geometry_msgs/PoseArray'
});

const waypointsdoneTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/global_mission/waypointsdone',
    messageType: 'geometry_msgs/PoseArray'
});




function startCleaning(scheduledCleaningIndex = null) {
    if (scheduledCleaningIndex !== null) {
        console.log('Starting cleaning for:', scheduledCleanings[scheduledCleaningIndex]);
        scheduledCleanings.splice(scheduledCleaningIndex, 1);
        broadcastScheduledCleanings();
    } else {
        console.log('Starting immediate cleaning.');
    }

    // Define the service client
    var startCleaningClient = new ROSLIB.Service({
        ros: ros,
        name: '/start_navigation',
        serviceType: 'hri_communication/StartNavigation'
    });

    // Create a request (Empty service has no arguments)
    var request = new ROSLIB.ServiceRequest({});

    // Call the service
    startCleaningClient.callService(request, function(result) {
        console.log('Result for service call on /start_navigation:', result);
        if (result.success) {
            isCleaning = true;
            subscribeToProgressTopics();
        }
    });    
    console.log('Cleaning session started.'); 
    
    subscribeToProgressTopics();
}

// Function to subscribe to waypoints topics
function subscribeToProgressTopics() {
    waypointsTopic.subscribe(message => {
    	ws.send(JSON.stringify({op: 'waypoints'}));        
        totalWaypoints = message.poses.length;
    });

    waypointsdoneTopic.subscribe(message => {
        waypointsDone = message.poses.length;
    });
}

// Function to stop cleaning and unsubscribe from topics
function stopCleaning() {
    isCleaning = false;
    waypointsTopic.unsubscribe();
    waypointsdoneTopic.unsubscribe();
    // Optionally reset the progress bar here if needed
}



function sendScheduledCleanings(ws) {
    const message = JSON.stringify({ op: 'update_scheduled_cleanings', scheduledCleanings });
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(message);
    }
}

function addScheduledCleaning(date, time) {
    scheduledCleanings.push({ date, time });
    scheduledCleanings.sort((a, b) => new Date(`${a.date}T${a.time}`) - new Date(`${b.date}T${b.time}`));
    broadcastScheduledCleanings();
}

function broadcastScheduledCleanings() {
    const message = JSON.stringify({ op: 'update_scheduled_cleanings', scheduledCleanings });
    wss.clients.forEach(client => {
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
            ws.send(JSON.stringify({command: 'schedulematch'}));        
            console.log('Scheduled cleaning matched. Starting cleaning...');
            // Match found, trigger cleaning
            startCleaning(index); // Pass the index of the matched cleaning
        }
    });
}

setInterval(checkScheduledCleanings, 10000);
////////////////////////////////////CLEANING/////////////////////////////////////

/////////////////////////////////////MAPPING//////////////////////////////////////

function startMapping() {
  // Define the service client
  var startMappingClient = new ROSLIB.Service({
    ros: ros,
    name: '/start_mapping',
    serviceType: 'hri_communication/StartMapping'
  });

  // Create a request (Empty service has no arguments)
  var request = new ROSLIB.ServiceRequest({});
  
  // Call the service
  startMappingClient.callService(request, function(result) {
    console.log('Result for service call on /start_mapping:', result);
  });
}


function stopMapping() {
// Define the service client
	var stopMappingClient = new ROSLIB.Service({
		ros: ros,
		name: '/stop_mapping',
		serviceType: 'std_srvs/Empty'
	});

	    // Create a request (Empty service has no arguments)
	var request = new ROSLIB.ServiceRequest({});

	// Call the service
	stopMappingClient.callService(request, function(result) {
		console.log('Result for service call on /stop_mapping:', result);
	});  
}


/////////////////////////////////////MAPPING//////////////////////////////////////

