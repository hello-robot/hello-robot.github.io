

function lookLeft() {
    var cmd = {type:"command",
               subtype:"head",
               name:"left",
               modifier:"medium"};
    sendData(cmd);
}

function lookRight() {
    var cmd = {type:"command",
               subtype:"head",
               name:"right",
               modifier:"medium"};
    sendData(cmd);
}

function lookUp() {
    var cmd = {type:"command",
               subtype:"head",
               name:"up",
               modifier:"medium"};
    sendData(cmd);
}

function lookDown() {
    var cmd = {type:"command",
               subtype:"head",
               name:"down",
               modifier:"medium"};
    sendData(cmd);
}

function moveForwardMedium() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"forward",
               modifier:"medium"};
    sendData(cmd);
}

function moveForwardSmall() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"forward",
               modifier:"small"};
    sendData(cmd);
}

function moveBackwardMedium() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"backward",
               modifier:"medium"};
    sendData(cmd);
}

function moveBackwardSmall() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"backward",
               modifier:"small"};
    sendData(cmd);
}

function turnLeftMedium() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"turn_left",
               modifier:"medium"};
    sendData(cmd);
}

function turnLeftSmall() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"turn_left",
               modifier:"small"};
    sendData(cmd);
}

function turnRightMedium() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"turn_right",
               modifier:"medium"};
    sendData(cmd);
}

function turnRightSmall() {
    var cmd = {type:"command",
               subtype:"drive",
               name:"turn_right",
               modifier:"small"};
    sendData(cmd);
}

function liftUpMedium() {
    var cmd = {type:"command",
               subtype:"lift",
               name:"up",
               modifier:"medium"};
    sendData(cmd);
}

function liftUpSmall() {
    var cmd = {type:"command",
               subtype:"lift",
               name:"up",
               modifier:"small"};
    sendData(cmd);
}

function liftDownMedium() {
    var cmd = {type:"command",
               subtype:"lift",
               name:"down",
               modifier:"medium"};
    sendData(cmd);
}

function liftDownSmall() {
    var cmd = {type:"command",
               subtype:"lift",
               name:"down",
               modifier:"small"};
    sendData(cmd);
}

function armRetractMedium() {
    var cmd = {type:"command",
               subtype:"arm",
               name:"retract",
               modifier:"medium"};
    sendData(cmd);
}

function armRetractSmall() {
    var cmd = {type:"command",
               subtype:"arm",
               name:"retract",
               modifier:"small"};
    sendData(cmd);
}

function armExtendMedium() {
    var cmd = {type:"command",
               subtype:"arm",
               name:"extend",
               modifier:"medium"};
    sendData(cmd);
}

function armExtendSmall() {
    var cmd = {type:"command",
               subtype:"arm",
               name:"extend",
               modifier:"small"};
    sendData(cmd);
}


function wristVelocityBend(degPerSec) {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"bend_velocity",
               modifier:degPerSec};
    sendData(cmd);
}

function gripperSetGoal(goalWidthCm) {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"set_goal",
               modifier:goalWidthCm};
    sendData(cmd);
}

function gripperClose() {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"close",
               modifier:"medium"};
    sendData(cmd);
}

function gripperOpen() {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"open",
               modifier:"medium"};
    sendData(cmd);
}

function gripperCloseFull() {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"fully_close",
               modifier:"medium"};
    sendData(cmd);
}

function gripperOpenHalf() {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"half_open",
               modifier:"medium"};
    sendData(cmd);
}

function gripperOpenFull() {
    var cmd = {type:"command",
               subtype:"gripper",
               name:"fully_open",
               modifier:"medium"};
    sendData(cmd);
}

function wristMotionStop() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"stop_all_motion",
               modifier:"medium"};
    sendData(cmd);
}

function wristVelocityBend(deg_per_sec) {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"bend_velocity",
               modifier:deg_per_sec};
    sendData(cmd);
}

function wristBendDown() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"bend_down",
               modifier:"medium"};
    sendData(cmd);
}

function wristBendUp() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"bend_up",
               modifier:"medium"};
    sendData(cmd);
}


function wristIn() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"in",
               modifier:"medium"};
    sendData(cmd);
}

function wristOut() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"out",
               modifier:"medium"};
    sendData(cmd);
}

function wristBendAuto(ang_deg) {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"auto_bend",
               modifier:ang_deg};
    sendData(cmd);
}

function wristRollRight() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"roll_right",
               modifier:"medium"};
    sendData(cmd);
}

function wristRollLeft() {
    var cmd = {type:"command",
               subtype:"wrist",
               name:"roll_left",
               modifier:"medium"};
    sendData(cmd);
}


//var cameraToVideoMapping = {nav: 'big', arm: 'smallTop', hand: 'smallBot'};
var interfaceMode = 'nav';
var interfaceModifier = 'no_wrist';


function turnModeOn(modeKey) {
    console.log('turnModeOn: modeKey = ' + modeKey)
    var cmd;
    if(noWristOn === false) {
	cmd = {type:"command",
               subtype:"mode",
               name : modeKey,
               modifier:"none"};
	interfaceModifier = 'none';
    } else {
	cmd = {type:"command",
               subtype:"mode",
               name : modeKey,
               modifier:"no_wrist"};
	interfaceModifier = 'no_wrist';
    }
    interfaceMode = modeKey
    sendData(cmd)
    turnModeUiOn(modeKey)
}

modeKeys = ['nav', 'low_arm', 'high_arm', 'hand', 'look']

function createModeCommands() {
    modeCommands = {}
    for (var index in modeKeys) {
	var key = modeKeys[index]
	// function inside a function used so that commandKey will not
	// change when key changes. For example, without this, the
	// interface mode and robotModeOn commands use key = 'look'
	// (last mode) whenever a function is executed.
	modeCommands[key] = function(commandKey) {
	    return function(modifier) {
		if(modifier === 'no_wrist') {
		    interfaceModifier = 'no_wrist';
		} else {
		    if(modifier !== 'none') {
		    console.log('ERROR: modeCommands modifier unrecognized = ', modifier);
		    }
		    interfaceModifier = 'none';
		}
		console.log('mode: command received with interfaceModifier = ' + interfaceModifier + ' ...executing');
		interfaceMode = commandKey
		robotModeOn(commandKey)
	    } 
	} (key)
    }
    return modeCommands
} 

var modeCommands = createModeCommands()


function executeCommandBySize(size, command, smallCommandArgs, mediumCommandArgs) {
    switch(size) {
    case "small":
        command(...smallCommandArgs);
        break;
    case "medium":
        command(...mediumCommandArgs);
        break;
    default:
        console.log('executeCommandBySize: size unrecognized, so doing nothing');
        console.log('executeCommandBySize: size = ' + size);
    }
}


var headCommands = {
    "up": function(size) {
        console.log('head: up command received...executing');
	headTilt(0.1)
    },
    "down": function(size) {
        console.log('head: down command received...executing');
	headTilt(-0.1)
    },
    "left": function(size) {
        console.log('head: left command received...executing');
	headPan(0.1)
    },
    "right": function(size) {
        console.log('head: right command received...executing');
	headPan(-0.1)
    }
}  

var driveCommands = {
    "forward": function(size) {
        console.log('drive: forward command received...executing');
	
        // executeCommandBySize(size, baseTranslate,
        //                      [-1.0, 10.0], // -1cm at 10 cm/s
        //                      [-10.0, 40.0]); // -10cm at 40 cm/s

        executeCommandBySize(size, baseTranslate,
                             [-10.0, 200.0], //dist (mm), speed (mm/s)
                             [-100.0, 200.0]); //dist (mm), speed (mm/s)
	
    },
    "backward": function(size) {
        console.log('drive: backward command received...executing');

	// executeCommandBySize(size, baseTranslate,
        //                      [1.0, 10.0], // 1cm at 10 cm/s
        //                      [10.0, 40.0]); // 10cm at 40 cm/s
	
        executeCommandBySize(size, baseTranslate,
                             [10.0, 200.0], //dist (mm), speed (mm/s)
                             [100.0, 200.0]); //dist (mm), speed (mm/s)
    },
    "turn_right": function(size) {
        console.log('drive: turn_right command received...executing');

	// executeCommandBySize(size, baseTurn,
        //                      [1.0, 10.0], // 1deg at 10 cm/s wheel velocity 
        //                      [10.0, 20.0]); // 10deg at 20 cm/s wheel velocity 

        executeCommandBySize(size, baseTurn,
                             [1.0, 300.0], // angle (deg), angular speed (deg/s)
                             [10.0, 300.0]); // angle (deg), angular speed (deg/s)

    },
    "turn_left": function(size) {
        console.log('drive: turn_left command received...executing');
        // executeCommandBySize(size, baseTurn,
        //                      [-1.0, 10.0], // -1deg at 10 cm/s wheel velocity 
        //                      [-10.0, 20.0]); // -10deg at 20 cm/s wheel velocity

	executeCommandBySize(size, baseTurn,
                             [-1.0, 300.0], // angle (deg), angular speed (deg/s)
                             [-10.0, 300.0]); // angle (deg), angular speed (deg/s)
    }
}  

var liftCommands = {
    "up": function(size) {
        console.log('lift: up command received...executing');

	// executeCommandBySize(size, lift,
        //                      [1.0, 10.0], // 1cm at 10 cm/s
        //                      [5.0, 20.0]); // 5cm at 30 cm/s
	
	executeCommandBySize(size, liftMove,
                             [10.0, -1], // dist (mm), timeout (s)
                             [100.0, -1]); // dist (mm), timeout (s)
   
	
    },
    "down": function(size) {
        console.log('lift: down command received...executing');
	
        // executeCommandBySize(size, lift,
        //                      [-1.0, 10.0], // -1cm at 10 cm/s
        //                      [-5.0, 20.0]); // -5cm at 30 cm/s

	executeCommandBySize(size, liftMove,
                             [-10.0, -1], // dist (mm), timeout (s)
                             [-100.0, -1]); // dist (mm), timeout (s)

    }
}  

var armCommands = {
    "extend": function(size) {
        console.log('arm: extend command received...executing');
        // executeCommandBySize(size, arm,
        //                      [1.0, 10.0], // 1cm at 10 cm/s
        //                      [5.0, 20.0]); // 5cm at 20 cm/s

	executeCommandBySize(size, armMove,
                             [10.0, -1], // dist (mm), timeout (s)
                             [100.0, -1]); // dist (mm), timeout (s)
    },
    "retract": function(size) {
        console.log('arm: retract command received...executing');
        // executeCommandBySize(size, arm,
        //                      [-1.0, 10.0], // -1cm at 10 cm/s
        //                      [-5.0, 20.0]); // -5cm at 20 cm/s

	executeCommandBySize(size, armMove,
                             [-10.0, -1], // dist (mm), timeout (s)
                             [-100.0, -1]); // dist (mm), timeout (s)

    }
}  


var wristCommands = {
    "in": function(nothing) {
	console.log('wrist: wrist_in command received...executing');
	wristMove(0.1)
    },
    "out": function(nothing) {
	console.log('wrist: wrist_out command received...executing');
	wristMove(-0.1)
    },    
    "stop_all_motion": function(nothing) {
	console.log('wrist: stop all motion command received...executing');
	wristStopMotion();
    },
    "bend_velocity": function(deg_per_sec) {
	console.log('wrist: bend velocity of ' + deg_per_sec + ' command received...executing');
	wristBendVelocity(deg_per_sec);
    },
    "auto_bend": function(ang_deg) {
	console.log('wrist: auto bend to ' + ang_deg + ' command received...executing');
	wristAutoBend(ang_deg);
    },
    "init_fixed_wrist": function(size) {
	console.log('wrist: init_fixed_wrist command received...executing');
	initFixedWrist();
    },
    "bend_up": function(size) {
        console.log('wrist: bend_up command received...executing');
        wristBend(5.0); // attempt to bed the wrist upward by 5 degrees
    },
    "bend_down": function(size) {
        console.log('wrist: bend_down command received...executing');
        wristBend(-5.0); // attempt to bed the wrist downward by 5 degrees
    },
    "roll_left": function(size) {
        console.log('wrist: roll_left command received...executing');
        wristRoll(-5.0); // attempt to roll the wrist to the left (clockwise) by 5 degrees
    },
    "roll_right": function(size) {
        console.log('wrist: roll_right command received...executing');
        wristRoll(5.0); // attempt to roll the wrist to the right (counterclockwise) by 5 degrees
    }
}

var gripperCommands = {
    "set_goal": function(goalWidthCm) {
        console.log('gripper: set_goal command received...executing');
        gripperGoalAperture(goalWidthCm); 
    },
    "open": function(size) {
        console.log('gripper: open command received...executing');
        gripperDeltaAperture(1.0); // attempt to increase the gripper aperature width by one unit
    },
    "close": function(size) {
        console.log('gripper: close command received...executing');
        gripperDeltaAperture(-1.0); // attempt to decrease the gripper aperature width by one unit
    },
    "fully_close": function(size) {
	console.log('gripper: fully close command received...executing');
	gripperFullyClose();
    },
    "half_open": function(size) {
	console.log('gripper: half open command received...executing');
	gripperHalfOpen();
    },
    "fully_open": function(size) {
	console.log('gripper: fully open command received...executing');
	gripperFullyOpen();	
    }
}

var commands = {
    "drive": driveCommands,
    "lift": liftCommands,
    "arm": armCommands,
    "wrist": wristCommands,
    "gripper": gripperCommands,
    "head": headCommands,
    "mode": modeCommands
}

function executeCommand(obj) {
    if ("type" in obj) {
        if (obj.type === "command") {
            commands[obj.subtype][obj.name](obj.modifier);
            return;
        }
    }
    console.log('ERROR: the argument to executeCommand was not a proper command object: ' + obj); 
}
