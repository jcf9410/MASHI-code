/*

2015/03/12: Adding REST operation to head motion

2016/12/21: Adding path-finding, path following & localization

create function headRest

*/
/////////////////////////////
// Base motion operations (Arduino)
/////////////////////////////
var EMER =	0X01;
var RARM =	0X02;
var VELO =	0X03;
var SMON =	0X04;
var SMOF =	0X05;
var PIDC =	0X06;
var PIDA =	0X07;

var offset = 100;
/////////////////////////////
// Head and arms motion operations (Robotis)
/////////////////////////////
var REST =  0X00;
var ZERO = 	0X01;
var POSI =	0X02;
var WAKE =	0X03;
var ARMS =	0X04;
var UP_LONG = 0x05;
var DOWN_LONG  =  0x06;
var UP_SHORT  =   0x07 ;
var DOWN_SHORT =  0x08;

//var TEST_HEAD =	0X05;


var DEBUG = 0X00; // Debug Print.  Set to FF to debug

var wheelRadius = 20;
var wheelDistance = 40;

var PF = require('pathfinding');
var emergency = false;
var coords = [0,0,Math.PI/2];
var bezier_curve = require('bezier').prepare(3);
var bezier_speed= require('bezier').prepare(2);
var bezier_acceleration = require('bezier').prepare(1);
if (typeof require !== 'undefined') XLSX = require('xlsx');
var workbook = XLSX.readFile('test.xlsx');
var sheet_name_list = workbook.SheetNames;
var inside = require('point-in-polygon');

var Twitter = require('twitter');

var client = new Twitter({
//Mashi robot account
  consumer_key: 'zMvhUCpHBY30OMNU343mkYnz3',
  consumer_secret: 'zTakxbMyGffcN05Df6RtK5kkEVpzvyW4Bm2iCZW2iZD09B9Ugz',
  access_token_key: '4163121033-NIsZ9qEshL9GfgaxGxb3S6Dnkb16vPKeLyKBq8s',
  access_token_secret: '1WQsWubxfCtlgcPfsVJnlEEpWabHEK2PizevC28pVTvXN'
});

var maxSocketsClients = 2;
var flagEventOn = false;

var fs = require('fs');
var express = require('express');
var atob = require('atob');
var http = require('http');
var https = require('https');

var privateKey = fs.readFileSync('fakekeys/privatekey.pem').toString();
var certificate = fs.readFileSync('fakekeys/certificate.pem').toString();

var app = express();

app.use(express.static(__dirname));

var server = https.createServer({key: privateKey, cert: certificate}, app).listen(8000);

var keypress = require('keypress');
var SerialPortArduino = require("serialport").SerialPort
var serialPortArduino = new SerialPortArduino("COM4", {baudrate: 115200}, false); // this is the openImmediately flag [default is true]
//var serialPortArduino = new SerialPortArduino("COM11", {baudrate: 57600}, false); // this is the openImmediately flag [default is true]

var SerialPortROBOTIS = require("serialport").SerialPort
var serialPortROBOTIS = new SerialPortROBOTIS("COM3", {baudrate: 1000000}, false); // this is the openImmediately flag [default is true]
//var serialPortROBOTIS = new SerialPortROBOTIS("COM9", {baudrate: 57600}, false); // this is the openImmediately flag [default is true]

//Buffer to send values through serialport
var bufferHeadSize = 4;
var bufferBaseSize = 4;
bufferHead = new Buffer(bufferHeadSize);
bufferBase = new Buffer(bufferBaseSize);
bufferBase_up = new Buffer(bufferBaseSize);
bufferBase_left = new Buffer(bufferBaseSize);
bufferBase_right = new Buffer(bufferBaseSize);
bufferBase_down = new Buffer(bufferBaseSize);


var idMotor;
var idOp; 
var pos;
var vel;
var nparam; //number of parameters basecommand


var varArmLeft;
var varArmRight;
var roll

// 128Kb Chunks
// var targetSize = 131072;


console.log('Corriendo en https://localhost:8000');

var io = require('socket.io').listen(server);

io.sockets.on('connection', function (socket){

	function log(){
		var array = [">>> Message from server: "];
		for (var i = 0; i < arguments.length; i++) {
	  	array.push(arguments[i]);
		}
	    socket.emit('log', array);
	}

	socket.on('message', function (message) {
		log('socket.on message: ', message);
    // For a real app, should be room only (not broadcast)
		socket.broadcast.emit('message', message);
        //io.sockets.in('robotRoom').emit('message', message);
	});

	socket.on('create or join', function (room) {
		var numClients = io.sockets.clients(room).length;
		console.log('Room ' + room + ' have ' + numClients + ' clients');
		if (numClients == 0){
			console.log('Client Robot');
			console.log('Socket.id: ' + socket.id + '; Room: ' + room);
			socket.join(room); // el primero que ingresa crea el cuarto
			socket.emit('created', room);
			allSockets.addSocket(socket, 'Robot');
		} else if (numClients < maxSocketsClients) {
			console.log('Client Operator');
			io.sockets.in(room).emit('join', room);
			socket.join(room); 
			socket.emit('joined', room); // se ha unido al grupo
			allSockets.addSocket(socket, 'Operator');
			flagEventOn = true;

		} else { // full
			console.log('Full');
			socket.emit('full', room);
		}
		//socket.emit('Client: ' + socket.id + '; Room: ' + room);
		//socket.broadcast.emit('broadcast(): client ' + socket.id + ' joined room ' + room);

	});

	socket.on('ACTUAR', function (message) {
		//console.log('socket.on ACTUAR: ', message);
        console.log('ACTUAR...');
        /* Values that are received from the socket */
        var matrixValuesVel = message.split( ";" );
        if (matrixValuesVel == ['ACTUAR:',1, 'VELO', 100, 100]){
            emergency = true;
        }
        var iBuffer = 0;
		nparam = matrixValuesVel[ 0 ];
		console.log(nparam);
		while (iBuffer<nparam){
			bufferBase[iBuffer] = matrixValuesVel[iBuffer+1];
			iBuffer++;
		}
        console.log('Op: '+ bufferBase[0])
        /* Send the information to the Arduino */
		baseCommand(bufferBase);
        });
		
	socket.on('HEADMO', function (message) {
        
        console.log('HEADMO...');
        /* Values that are received from the socket */
        var matrixValues = message.split( ";" );
        
        /* Select each one of the values for the movement of the head */
        varPitch = matrixValues[ 0 ];
        varYaw = matrixValues[ 1 ];
        varRoll = matrixValues[ 2 ];
        
        console.log('p: ' + varPitch + ' ' + varYaw + ' ' + varRoll )
        /* Send the information to the Robotis */
		headMovement(varPitch , varYaw , varRoll);
		});
	socket.on('ARMSMO', function (message) {
        
        console.log('ARMSMO...');
        /* Values that are received from the socket */
        var matrixValues = message.split( ";" );
        
        /* Select each one of the values for the movement of the head */
        var1 = matrixValues[ 0 ];
        var2 = matrixValues[ 1 ];
        var3 = matrixValues[ 2 ];
        
        console.log('p: ' + var1 + ' ' + var2 + ' ' + var3 )
        /* Send the information to the Robotis */
		armMovement(var1 , var2 , var3);
		});
	socket.on('SHLDUP', function (message) { //shoulder
        
        console.log('SHLDUP...');
        /* Send the information to the Robotis */
		upLong();
		});
	socket.on('SHLDDO', function (message) { //shoulder
        
        console.log('SHLDDO...');
        /* Send the information to the Robotis */
		downLong();
		});
	socket.on('HEADWR', function (message) {
        
        console.log('HEADWR...');
        /* Values that are received from the socket */
        if(message=="REST"){
			headRest();

		}else if(message=="WAKE")
			headWake();
	});

	socket.on('PATH-PLNG', function (message) {
	    console.log(message);
	    var start_goal = String(message).split(";");

	    // path-finding
	    best_path = path_finding(start_goal[0],start_goal[1], start_goal[2], start_goal[3]);
	    //llevar camino a funcion de seguimiento movimientos
	    path_following(best_path, emergency);
	});

	socket.on('TWEET', function (message) {
        
        console.log('TWEET');
        //console.log(message);
		//var data = atob(message);
		var data = message.substr(22,message.length)
		console.log(data);
		client.post('media/upload', {media_data: data}, function(error, media, response){
		  if (!error) {

			// If successful, a media object will be returned.
			console.log(media);

			// Lets tweet it
			var status = {
			  status: 'Welcome to L\'Hospitalet #SCEWC16 #smartcity #smartcities #citiesforcitizens',
			  media_ids: media.media_id_string // Pass the media id string
			}

			client.post('statuses/update', status, function(error, tweet, response){
			  if (!error) {
				console.log(tweet);
			  }
			  else
				  console.log('TWEET ERROR: statuses/update');
			});

		  }
		  else
			  console.log('TWEET ERROR: media/upload: '+error);
		});
		
		
		
	});
	var filesSounds = [];	
	socket.on('requestListSounds', function (message) {
		console.log('Server: requestListSounds');
		//List files of sounds in folder "audio"
		var pathAudio = require("path");
		var p = "../ws/audio"; 
		fs.readdir(p, function (err, files) {
			if (err) {
				throw err;
			}
/*			files.map(function (file) {
				return pathAudio.join(p, file);//return pathAudio.join(p, file);
			}).filter(function (file) {
			return fs.statSync(file).isFile();
			})*/
			files.forEach(function (file) {
				//console.log("%s (%s)", file, pathAudio.extname(file));
				//console.log(file);
				filesSounds.push(file);
				//console.log(filesSounds);
			});
		});
		console.log(filesSounds);
		//socket.emit('responseListSounds', filesSounds);JSON.stringify(val)
		socket.emit('responseListSounds', JSON.stringify(filesSounds));
		filesSounds = [];
	});		
});


var baseCommand = function(bufferBase) {
    console.log('baseCommand = ',bufferBase);
    serialPortArduino.write(bufferBase);
}

var headMovement = function(varPitch , varYaw , varRoll ) {
    idOp = POSI;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
    console.log('headMovement = ',bufferHead);
    serialPortROBOTIS.write(bufferHead);
    
}
var armMovement = function(varArmLeft , varArmRight, varDummy) {
    idOp = ARMS;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
    console.log('armMovement = ',bufferHead);
    serialPortROBOTIS.write(bufferHead);
    
}

///////////////////////////////////////////
// KEYPRESS EVENTS
///////////////////////////////////////////
keypress(process.stdin);
var keys = {
    'f1': function () {
        console.log('Emergency stop!');
		bufferBase[0] = EMER;
		// bufferBase[1] = 0 + offset;
		// bufferBase[2] = 0 + offset;
        baseCommand (bufferBase);
    },
    'f2': function () {
        console.log('Rearm motors!');
		bufferBase[0] = RARM;
		// bufferBase[1] = 0 + offset;
		// bufferBase[2] = 0 + offset;
        baseCommand (bufferBase);
    },
    'f3': function () {
        console.log('Social motion ON...');
		bufferBase[0] = SMON;
		// bufferBase[1] = 0 + offset;
		// bufferBase[2] = 0 + offset;
        baseCommand (bufferBase);
    },
    'f4': function () {
        console.log('Social motion OFF...');
		bufferBase[0] = SMOF;
		// bufferBase[1] = 0 + offset;
		// bufferBase[2] = 0 + offset;
        baseCommand (bufferBase);
        // baseCommand (SMOF, 0 + offset , 0 + offset );

    },
    'f5': function () {
        console.log('PID Conservative...');
		bufferBase[0] = PIDC;
        baseCommand (bufferBase);

    },
    'f6': function () {
        console.log('PID Aggresive...');
		bufferBase[0] = PIDA;
        baseCommand (bufferBase);

    },
    'up': function () {
        console.log('Forward!');
        // baseCommand (VELO, 30 + offset , 30 + offset );
		bufferBase[0] = VELO;
		bufferBase[1] = 10 + offset;
		bufferBase[2] = 10 + offset;
        baseCommand (bufferBase);
    },
    'down': function () {
        console.log('Reverse!');
        // baseCommand (VELO, -15 + offset, -15 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = -10 + offset;
		bufferBase[2] = -10 + offset;
        baseCommand (bufferBase);

    },
    'left': function () {
        console.log('Turn left!');
        // baseCommand (VELO, -15 + offset, 15 + offset);    
		bufferBase[0] = VELO;
		bufferBase[1] = -15/2 + offset;
		bufferBase[2] = 15/2 + offset;
        baseCommand (bufferBase);
		
	},
    'right': function () {
        console.log('Turn right!');
        // baseCommand (VELO, 15 + offset, -15 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = 15/2 + offset;
		bufferBase[2] = -15/2 + offset;
        baseCommand (bufferBase);
		
	},
    'space': function () {
        console.log('STOP!');
        // baseCommand (VELO, 0 + offset, 0 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = 0 + offset;
		bufferBase[2] = 0 + offset;
		baseCommand(bufferBase);
		emergency = true;
		
    },
	///////////////////////////////////////
	// HEAD MOTION
	///////////////////////////////////////
  	// PITCH
    'w': function () {
        //console.log('SERVER KEY PITCHUP');
        pitchUp();
    },
    'x': function () {
        //console.log('SERVER KEY PITCHDOWN');
		pitchDown();
    },
	// YAW
    'a': function () {
        //console.log('SERVER KEY YAWLEFT');
		yawLeft();
    },
    'd': function () {
        console.log('SERVER KEY YAWRIGHT');
		yawRight();
    },
	// ROLL
    'q': function () {
		rollLeft();
    },
    'e': function () {
		rollRight();
    },
	// ARMS
    'f': function () {
		armLeftUp();
    },
    'v': function () {
		armLeftDown();
    },
    'j': function () {
		armRightUp();
    },
    'm': function () {
		armRightDown();
    },
    'o': function () {
		upLong();
    },
    'l': function () {
		downLong();
    },
	// // TESTING, ZEROING, WAKING, RESTING
    's': function () {
	headZero();
    },
    // 't': function () {
		// headRest();
    // },
    // 'g': function () {
		// headWake();
    // },
    'y': function () { // testing path finding
        /*
        var start = [650, 10];
        var goal = [650, 650];
        var pathToFollow = path_finding(start, goal);
        coords[0] = 650;
        coords[1] = 10;
        path_following(pathToFollow);
        */
     }
}

/////////////////////////////////////////////////////
// BASE COMMAND SPECIAL FUNCTIONS
/////////////////////////////////////////////////////
// var ledOn = function () {
    // console.log('ledOn');
    // serialPortArduino.write("6");
// }

// var ledOff = function () {
    // console.log('ledOff');
    // serialPortArduino.write("7");
// }


// var ledBlink = function () {
    // console.log('ledBlink');
    // serialPortArduino.write("8");
// }

// var socialMotionTrue = function () {
    // console.log('ledBlink');
    // serialPortArduino.write("9");
// }
// var socialMotionFalse = function () {
    // console.log('ledBlink');
    // serialPortArduino.write("10");
// }
/////////////////////////////////////////////////////

// HEAD MOTION FUNCTIONS
/////////////////////////////////////////////////////
var pitchUp = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 70;
	varYaw = 50;
	varRoll = 50;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server pitchUp = ',bufferHead);}			
    serialPortROBOTIS.write(bufferHead);
}
var pitchDown = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 0;
	varYaw = 50;
	varRoll = 50;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server pitchDown = ',bufferHead);}			
    serialPortROBOTIS.write(bufferHead);
}

var yawLeft = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 50;
	varYaw = 100;
	varRoll = 50;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server yawLeft = ',bufferHead);}			
    serialPortROBOTIS.write(bufferHead);
}
var yawRight = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 50;
	varYaw = 0;
	varRoll = 50;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server yawRight = ',bufferHead);}		
    serialPortROBOTIS.write(bufferHead);
}

var rollLeft = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 50;
	varYaw = 50;
	varRoll = 100;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server rollLeft = ',bufferHead);}		
    serialPortROBOTIS.write(bufferHead);
}
var rollRight = function (varPitch, varYaw, varRoll) {
    idOp = POSI; //operation 0x02
	varPitch = 50;
	varYaw = 50;
	varRoll = 0;
    bufferHead[0] = idOp;
    bufferHead[1] = varPitch;
    bufferHead[2] = varYaw;
    bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server rollRight = ',bufferHead);}	
    serialPortROBOTIS.write(bufferHead);
}

var armLeftUp = function (varArmLeft, varArmRight,varRoll) {
    idOp = ARMS;
	varArmLeft = 100;
	//varArmRight = 80;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;	
	if (DEBUG){console.log('Server armLeftUp = ',bufferHead);}	
    serialPortROBOTIS.write(bufferHead);
}

var armLeftDown = function (varArmLeft, varArmRight,varRoll) {
    idOp = ARMS;
	varArmLeft = 30;
	//varArmRight = 60;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;	
	if (DEBUG){console.log('Server armLeftDown = ',bufferHead);}	
    serialPortROBOTIS.write(bufferHead);
}
var armRightUp = function (varArmLeft, varArmRight,varRoll) {
    idOp = ARMS;
	varArmRight = 100;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server armRightUp = ',bufferHead);}	
    serialPortROBOTIS.write(bufferHead);
}

var armRightDown = function (varArmLeft, varArmRight,varRoll) {
    idOp = ARMS;
	varArmRight = 30;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;	
	if (DEBUG){console.log('Server armRightDown = ',bufferHead);}	
    serialPortROBOTIS.write(bufferHead);
}

var upLong = function (varArmLeft, varArmRight,varRoll) {
    idOp = UP_LONG;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;	
    serialPortROBOTIS.write(bufferHead);
}
var downLong = function (varArmLeft, varArmRight,varRoll) {
    idOp = DOWN_LONG;
    bufferHead[0] = idOp;
    bufferHead[1] = varArmLeft;
    bufferHead[2] = varArmRight;
	bufferHead[3] = varRoll;	
    serialPortROBOTIS.write(bufferHead);
}



var headZero = function (varPitch, varYaw, varRoll) {
	idOp = ZERO; //Zero operation
	varPitch = 50;
	varYaw = 50;
	varRoll = 50;
	bufferHead[0] = idOp;
	bufferHead[1] = varPitch;
	bufferHead[2] = varYaw;
	bufferHead[3] = varRoll;
	if (DEBUG){console.log('Server headZero = ',bufferHead);}
    serialPortROBOTIS.write(bufferHead);
}

var headRest = function () {
	idOp = REST; //Zero operation
	bufferHead[0] = idOp;
	if (DEBUG){console.log('Server headRest = ',bufferHead);}
    serialPortROBOTIS.write(bufferHead);
}

var headWake = function () {
	idOp = WAKE; //Zero operation
	bufferHead[0] = idOp;
	if (DEBUG){console.log('Server headWake = ',bufferHead);}
    serialPortROBOTIS.write(bufferHead);
}

var headTest = function () {
	idOp = TEST_HEAD; //Zero operation
	bufferHead[0] = idOp;
	if (DEBUG){console.log('Server send TEST_HEAD = ',bufferHead);}
    serialPortROBOTIS.write(bufferHead);
}


console.log("Iniciando keypress...");
process.stdin.on('keypress', function (ch, key) {
	//console.log(key);
    if (key && keys[key.name]) { keys[key.name](); }
    if (key && key.ctrl && key.name == 'c') { quit(); }
});

process.stdin.setRawMode(true);
process.stdin.resume();

/////////////////////////////////
// OPENING SERIAL PORTS
/////////////////////////////////
serialPortArduino.open(function () {
    console.log('Server: serialport.open');
    serialPortArduino.on('data', function (data) {
        //console.log("Arduino:" + data);
        /*
        if (String(data).length>=22 && (String(data).indexOf("|X") == 0)){
            coords = (String(data).slice(String(data).indexOf("|X") + 1, String(data).indexOf("||"))).split("|");
            for (var i = 0; i < coords.length; i++){
                coords[i] = parseFloat(coords[i].slice(2));
            }
            coords[0] = parseInt(coords[0]*100);
            coords[1] = parseInt(coords[1]*100);
        }
        console.log(coords);
        */
        //console.log("Arduino: " + data);
		//console.log('Arduino->Server: ');
		//console.log(data);
    });
});
serialPortROBOTIS.open(function () {
    console.log('Server: serialportROBOTIS.open');
    serialPortROBOTIS.on('data', function (data) {
        console.log('Robotis->Server: ' + data);
		data = data+'';
		if (data.substr(0, 5) === 'EVENT'){
			//console.log(user + ' ACTUAR ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
			if (flagEventOn){
				var socket = allSockets.getSocketByName('Operator');
				//socket.emit('EVENT', data + '');
				socket.emit('event', data.substr(6,data.length));
			}
			//    socket.emit('ACTUAR', dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
		}
        // var matrixValues = message.split( ";" );
        
        // /* Select each one of the values for the movement of the head */
        // varPitch = matrixValues[ 0 ];
        // varYaw = matrixValues[ 1 ];
        // varRoll = matrixValues[ 2 ];
		
		
    });
});



var quit = function () {
    console.log('Server: Saliendo de keypress y serialport...');
    serialPortArduino.close();
    serialPortROBOTIS.close();
    process.stdin.pause();
    process.exit();
}

var allSockets = {

  // A storage object to hold the sockets
  sockets: {},

  // Adds a socket to the storage object so it can be located by name
  addSocket: function(socket, name) {
    this.sockets[name] = socket;
  },

  // Removes a socket from the storage object based on its name
  removeSocket: function(name) {
    if (this.sockets[name] !== undefined) {
      this.sockets[name] = null;
      delete this.sockets[name];
    }
  },

  // Returns a socket from the storage object based on its name
  // Throws an exception if the name is not valid
  getSocketByName: function(name) {
    if (this.sockets[name] !== undefined) {
      return this.sockets[name];
    } //else {"oes not exist");
  }
    /*
      throw new Error("A socket with the name '"+name+"' d)
  }
  */
};

var maze = [];
var row = [];
sheet_name_list.forEach(function (y) { /* iterate through sheets */
    var worksheet = workbook.Sheets[y];
    for (z in worksheet) {
        if (JSON.stringify(z)[1] == "Z" && JSON.stringify(z)[2] == "X") { //modify according to last columm of excel file
            maze.push(row);
            row = [];
        }
        else {
            if (worksheet[z].v != 0) {
                row.push(0);
            }
            else {
                row.push(1);
            }
        }
        //console.log(JSON.stringify(worksheet[z].v));
    }/*
    console.log(maze);
    console.log("start", maze[699][50]);
    console.log("finish", maze[650][650]);*/
});

var bezierLengths = [];
var bezierPoints = [];
var bezierSpeeds = [];
var bezierRealTimes = [];
var bezierRadius = [];

var path_finding = function (start, goal) {
    //maze[y][x]
    //each element is 1 cm
    bezierLengths = [];
    bezierPoints = [];
    bezierSpeeds = [];
    bezierRealTimes = [];
    bezierRadius = [];
    var grid = new PF.Grid(maze);
    var finder = new PF.JumpPointFinder({
        allowDiagonal: true,
        dontCrossCorners: true
    });
    console.log("running...");
    var sx = start[0];
    var sy = start[1];
    var gx = goal[0];
    var gy = goal[1];
    var path = finder.findPath(sy, sx, gy, gx, grid);
    for (var lent = 0; lent < path.length - 1; lent++) {
        path[lent] = [path[lent][1], path[lent][0]];
    }
    var lastPoint = path[path.length-1];
    //transform array of points to smaller array of keypoints
    var keypoints = [path[0]];
    path.shift();
    var finish = false;
    while (!finish){ 
        //check colinearity of path[0], path[1] and path [2]. 
        var colinearity = ((path[0][1]-path[1][1])*(path[0][0]-path[2][0])==(path[0][1]-path[2][1])*(path[0][0]-path[1][0]));
        if (colinearity){
            path.splice(1,1);
        }
        else{
            keypoints.push(path[1]);
            path.shift();
        }
        if (path[2][0] == lastPoint[0] && path[2][1] == lastPoint[1]) {
            keypoints.push(path[2]);
            finish = true;
            break;
        }
    }
    console.log("keypoints", keypoints);
    /*
    var end2 = false;
    keypoints = path.slice(0, path.length);
    var keypoints2 = [keypoints[0]];
    var finalPoint = keypoints[keypoints.length - 1]
    var d = 10;
    while (!end2) {
        if (d == 0) {
            end2 = true;
            break;
        }
        //linearize path, eliminating short lines
        var uVector = [keypoints[1][0] - keypoints[0][0], keypoints[1][1] - keypoints[0][1]];
        var uModule = Math.sqrt(Math.pow(uVector[0], 2), Math.pow(uVector[1], 2));
        var uVectorNormal = [uVector[1]/uModule, -uVector[0]/uModule];
        var A = [keypoints[0][0] + d * uVectorNormal, keypoints[0][1] + d * uVectorNormal];
        var B = [keypoints[0][0] - d * uVectorNormal, keypoints[0][1] - d * uVectorNormal];
        var C = [A[0] + 100000 * uVector[0] / uModule, A[1] + 100000 * uVector[0] / uModule];
        var D = [B[0] + 100000 * uVector[0] / uModule, B[1] + 100000 * uVector[0] / uModule];
        var poly = [A, B, C, D];
        // console.log("final2", keypoints[keypoints.length - 1] == finalPoint);
        for (var j = 2; j < keypoints.length - 1; j++) {
            //console.log("j", j);
            // console.log("key", keypoints[j]);
            
            if (keypoints[j] == finalPoint) {
                end2 = true;
                keypoints2.push(keypoints[j]);
                break;
            }
            
            //sleep(1000);
            if (!inside(keypoints[j], poly)) {
                var v = [keypoints[j][0] - keypoints[0][0], keypoints[j][1] - keypoints[0][1]];
                for (var k = 0; k <= 1;) {
                    var point = [keypoints[0][0] + k * v[0], keypoints[0][1] + k * v[1]];
                    var i_maze = Math.floor(point[0]);
                    var j_maze = Math.floor(point[1]);
                    if (maze[i_maze][j_maze] == 1) {
                        end2 = false;
                        d = d - 1;
                        break;
                    }
                    k += 0.01;
                }
                if (d == 0) {
                    keypoints2.push(keypoints.slice(0,j+1));
                    keypoints = keypoints.slice(j, keypoints.length);
                    break;
                }
                if (k >= 1){
                    keypoints2.push(keypoints[j]);
                    keypoints = keypoints.slice(j, keypoints.length);
                    break;
                    //console.log("new keypoints to check:", keypoints.slice(j, keypoints.length));
                    //console.log("new points", keypoints2);
                }
                break;
            }
        }
        //console.log("finished poly loop", end2);
}
*/

    //check where bezier curves can be applied with restrictions
    // v =  0.5 m/s
    var p_start = [];
    var p_start_module = 0;
    var v_start = [];
    var P0 = [];
    var P1 = [];
    var p_end = [];
    var p_end_module = 0;
    var v_end = [];
    var P2 = [];
    var P3 = [];
    var i_maze = 0;
    var j_maze = 0;
    var beziered = false;
    var newPoints = [keypoints[0]];
    for (var j = 1; j<keypoints.length-1;j++){ //start and goal are final points of straight line
        //Calculate control points
        var A = keypoints[j-1];
        var B = keypoints[j];
        var C = keypoints[j + 1];
        //console.log("calculating k...");
        for (var k = 0.1; k<0.4;){ 
            p_start = [B[0]-A[0], B[1]-A[1]];
            p_start_module = Math.sqrt(Math.pow(p_start[0], 2)+Math.pow(p_start[1], 2));
            p_end = [C[0]-B[0], C[1]-B[1]];
            p_end_module = Math.sqrt(Math.pow(p_end[0], 2)+Math.pow(p_end[1], 2));
            P0 = [A[0]+(1-k)*(B[0]-A[0]),A[1]+(1-k)*(B[1]-A[1])];
            P3 = [B[0]+k*(C[0]-B[0]),B[1]+k*(C[1]-B[1])];
            //v = 0.5 m/s = 50 cm/s
            v_start = [50*p_start[0]/p_start_module, 50*p_start[1]/p_start_module];
            P1 = [P0[0]+v_start[0]*1/3,P0[1]+v_start[1]*1/3];
            v_end = [50*p_end[0]/p_end_module, 50*p_end[1]/p_end_module];
            P2 = [P3[0]-v_end[0]*1/3,P3[1]-v_end[1]*1/3];
            //check if max of Bdot >maxSpeed
            //maxSpeed = 1 m/s
            //console.log("calculating maxSpeed...");
            for (var timer = 0; timer < 1;){
                var x = bezier_curve([P0[0],P1[0],P2[0],P3[0]], timer);
                var y = bezier_curve([P0[1],P1[1],P2[1],P3[1]], timer);
                var speedy = bezier_speed([3*(P1[0]-P0[0]), 3*(P2[0]-P1[0]), 3*(P3[0]-P2[0])], timer);
                var speedx = bezier_speed([3*(P1[1]-P0[1]), 3*(P2[1]-P1[1]), 3*(P3[1]-P2[1])], timer);
                var speedMod = Math.sqrt(Math.pow(speedx, 2) + Math.pow(speedy, 2));
                if (speedMod < 100) {
                    //check min radius of curvatyre = 10 cm
                    var accely = bezier_acceleration([6 * (P2[0] - P1[0]) - P1[0] + P0[0], 6 * (P3[0] - P2[0] - P2[0] - P1[0])], timer);
                    var accelx = bezier_acceleration([6 * (P2[1] - P1[1]) - P1[1] + P0[1], 6 * (P3[1] - P2[1] - P2[1] - P1[1])], timer);
                    var radius = 1/(Math.abs(speedx * accely - speedy * accelx) / Math.pow(Math.pow(speedx, 2) + Math.pow(speedy, 2), 3 / 2));
                    if (radius > 10) {
                        //checkif curve is inside configuration space
                        i_maze = Math.floor(x);
                        j_maze = Math.floor(y);
                        if (maze[j_maze][i_maze] == 1) {
                            beziered = false;
                            break;
                        }
                        else {
                            beziered = true;
                            timer += 0.01;
                        }
                    }
                    else {
                        beziered = false;
                        break;
                    }
                }
                else {
                    //beziered = false;
                   break;
                }
            }
            if (beziered) {
                break;
            }
            k += 0.01;
        }
        if (beziered) {
            arcBezier(P0, P1, P2, P3);
            P0.push(true);
            P1.push(true);
            P2.push(true);
            P3.push(true);
            newPoints.push(P0);
            newPoints.push(P1);
            newPoints.push(P2);
            newPoints.push(P3);
            beziered =  false;
        }
        else {
            newPoints.push(keypoints[j]);
        }
    }
    newPoints.push(keypoints[keypoints.length - 1]);
    //console.log("path : ", newPoints);
    //console.log("times : ", bezierRealTimes);
    //console.log("lengths : ", bezierLengths);
    console.log("path : ", newPoints);
    return newPoints;
};

var arcBezier = function (a, b, c, d) {
    var points = [a];
    var arcLength = [0];
    var speedPoints = [];
    var curvature = [];
    time = 0;
    for (var t = 0.01; t <= 1;) {
        var By = bezier_curve([a[0], b[0], c[0], d[0]], t);
        var Bx = bezier_curve([a[1], b[1], c[1], d[1]], t);
        var ds = [By - points[points.length - 1][0], Bx - points[points.length - 1][1]];
        //console.log("points", [By, Bx]);
       // console.log("ds", ds);
        arcLength.push(arcLength[arcLength.length-1]+Math.sqrt(Math.pow(ds[0], 2) + Math.pow(ds[1], 2)));
        points.push([By, Bx]);
        var Bdoty = bezier_speed([3 * (b[0] - a[0]), 3 * (c[0] - b[0]), 3 * (d[0] - c[0])], t-0.01);
        var Bdotx = bezier_speed([3 * (b[1] - a[1]), 3 * (c[1] - b[1]), 3 * (d[1] - c[1])], t - 0.01);
        var BdotMod = Math.sqrt(Math.pow(Bdotx, 2) + Math.pow(Bdoty, 2));
        speedPoints.push([Bdoty/BdotMod, Bdotx/BdotMod]);
        time += (Math.sqrt(Math.pow(ds[0], 2) + Math.pow(ds[1], 2))) / (Math.sqrt(Math.pow(Bdotx, 2) + Math.pow(Bdoty, 2)));
        var accely = bezier_acceleration([6 * (c[0] - b[0]) - b[0] + a[0], 6 * (d[0] - c[0] - c[0] - b[0])], t-0.01);
        var accelx = bezier_acceleration([6 * (c[1] - b[1]) - b[1] + a[1], 6 * (d[1] - c[1] - c[1] - b[1])], t-0.01);
        var radius = 1 / (Math.abs(Bdotx * accely - Bdoty * accelx) / Math.pow(Math.pow(Bdotx, 2) + Math.pow(Bdoty, 2), 3 / 2));
        curvature.push(radius);
        t += 0.01;
    }
    bezierLengths.push(arcLength);
    bezierPoints.push(points);
    bezierSpeeds.push(speedPoints);
    //console.log("bezierPoints :", bezierPoints);
    //console.log("arcLengths: ", arcLength[arcLength.length-1]);
    bezierRealTimes.push(time);
    bezierRadius.push(curvature);
};

var up = function () {
    bufferBase[0] = VELO;
    bufferBase[1] = 10 + offset;
    bufferBase[2] = 10 + offset;
    //baseCommand(bufferBase);   //uncomment to write to Arduino
    //sim commands. Comment when using positioning
    coords[0] += 0.1*Math.cos(coords[2]);
    coords[1] += 0.1 * Math.sin(coords[2]);
    //console.log("buffer" + bufferBase);
}

var right = function(){
    bufferBase[0] = VELO;
    bufferBase[1] = 7.5 + offset;
    bufferBase[2] = -7.5 + offset;
    //baseCommand(bufferBase); //uncomment to write to Arduino
    //sim commands. Comment when using positioning
    coords[2] -= 0.01;
    //console.log("buffer" + bufferBase);
}

var left = function(){
    bufferBase[0] = VELO;
    bufferBase[1] = -7.5 + offset;
    bufferBase[2] = 7.5 + offset;
   // baseCommand(bufferBase); // //uncomment to write to Arduino
    //sim commands. Comment when using positioning
    coords[2] += 0.01;
    //console.log("buffer" + bufferBase);
}

var down = function(){
    var down_command = ['VELO', -120, -120];
    var i_down = 0;
    nparam_down = down_command[ 0 ];
    while (iBuffer_down<nparam_down){
        bufferBase_down[iBuffer_down] = down_command[iBuffer_down+1];
        iBuffer_down++;
    }
    baseCommand(bufferBase_down);
    //console.log("buffer" + bufferBase);
}

var stop = function(){
    bufferBase[0] = VELO;
    bufferBase[1] = 0 + offset;
    bufferBase[2] = 0 + offset;
    //baseCommand (bufferBase); //uncomment to write to Arduino
    //console.log("buffer" + bufferBase);
}


var writeSpeed = function (v) {
    //left, rigth
    bufferBase[0] = VELO;
    bufferBase[1] = v[0] + offset;
    bufferBase[2] = v[1] + offset;
    //baseCommand(bufferBase); //uncomment to write to Arduino
    //sim commands. Comment when using positioning

};

var inverse_odometry = function (Bdot, R) {
    var fiL = 2 * Math.sqrt(Math.pow(Bdot[0], 2) + Math.pow(Bdot[1], 2)) * (R / (R + wheelDistance - 1) / wheelRadius);
    var fiR = R*fiL/(wheelDistance+R);
    return [fiL, fiR];
};

var position = [];

var positionment = function () {
    position[0] = coords[0];
    position[1] = coords[1];
    position[2] = coords[2];
    return position;
};
    
var textFile = require('fs');
var s = "";

var path_following = function (path) {
    var previous_order = "";
    var path_copy = path.slice(0, path.length - 1);
    path_copy.shift();
    var exeTime = new Date().getTime();
    var emergency_stop = emergency;
    if (path_copy.length == 0) {
        console.log("no path was found!");
    }
    else {
        while (path_copy.length != 0) {
            var exeTimeTotal = new Date().getTime() - exeTime;
            actual_position = positionment();
            //console.log(actual_position);
            //s = s + coords[1].toString() + "\n";
            /*
            if (exeTimeTotal/1000 > 5) {
                console.log("exit!", path_copy);

                break;
            }
            */
            var t_vector = [path_copy[0][0] - actual_position[0], path_copy[0][1] - actual_position[1]];
            var t_module = Math.sqrt(Math.pow(t_vector[0], 2) + Math.pow(t_vector[1], 2));
            //console.log("t module", t_vector);
            if (t_module <= 0.1) {
                if (path_copy[0].length == 3) {
                    //follow bezier curve. Check position and speed
                    var bezierTime = new Date().getTime();
                    /*  // uncomment wher using real system
                    for (var timer = 0; timer <= bezierRealTimes[0];) {
                        timer = new Date().getTime() - bezierTime;
                        console.log("timer", timer);
                        var t_mod = 0.001* timer / bezierRealTimes[0];
                        var speedToWrite = bezierSpeeds[0][Math.floor(100 * t_mod)];
                        var localRadius = bezierRadius[0][Math.floor(100 * t_mod)];
                        var motorSpeed = inverse_odometry(speedToWrite, localRadius);
                        //writeSpeed(motorSpeed);  //uncomment to write to Arduino
                        //sim command. comment when using positioning
                        coords[0] = bezier_curve([path_copy[0][0], path_copy[1][0], path_copy[2][0], path_copy[3][0]], t_mod);
                        coords[1] = bezier_curve([path_copy[0][1],  path_copy[1][1], path_copy[2][1], path_copy[3][1]],  t_mod);
                        console.log("coordsBEzier: ", coords);
                        console.log("tmod", timer);
                    }
                    */
                    //sim loop
                    for (var i = 0; i < 99; i++) {
                        coords[0] = bezier_curve([path_copy[0][0], path_copy[1][0], path_copy[2][0], path_copy[3][0]], i/100);
                        coords[1] = bezier_curve([path_copy[0][1], path_copy[1][1], path_copy[2][1], path_copy[3][1]], i/100);
                        //console.log("bezierSpedd", bezierSpeeds[0][i][1]);
                        var bezierAngle = - Math.atan2(bezierSpeeds[0][i][1], bezierSpeeds[0][i][0]);
                        //console.log("bezierAngle", bezierAngle);
                        //coords[2] = bezierAngle;
                    }
                    //path_copy.slice(0, 4);
                    path_copy.shift();
                    path_copy.shift();
                    path_copy.shift();
                    coords[2] = Math.atan2(path_copy[1][1] - path[0][1], path_copy[1][0] - path[0][0]); //comment when using Arduino
                    path_copy.shift();
                    bezierSpeeds.shift();
                    bezierRadius.shift();
                    bezierRealTimes.shift();
                }
                else {
                    path_copy.shift();
                    stop();
                }
            }
            else {
                //check if robor orientates or goes straight
                var orientationAngle = actual_position[2];
                var angleT = Math.atan2(t_vector[1], t_vector[0]);
                var angle = orientationAngle - angleT;
                if (angle > 2 * Math.PI) {
                    angle -= 2 * Math.PI;
                }
               // console.log("angle", angle);
                if ((angle < 0.01 && angle > -0.01)) {
                    /*
                    if (previous_order != "up") {
                        stop();
                        sleep(100);
                    }
                    */
                    up();
                    //console.log("coords: ", coords);
                    //console.log("up");
                    previous_order = "up";
                }
                else if (angle<=Math.PI && angle>0) {
                    /*
                    if (previous_order != "right") {
                        stop();
                        sleep(100);
                    }
                    */
                    right();
                   // console.log("coords: ", coords);
                    previous_order = "right";
                }
                else {
                    /*
                    if (previous_order != "left") {
                        stop();
                        sleep(100);
                    }
                    */
                    left();
                    //console.log("coords: ", coords);
                    previous_order = "left";
                }
            }

        }
    }
    var t_vector = [path[path.length-1][0] - actual_position[0], path[path.length-1][1] - actual_position[1]];
    var t_module = Math.sqrt(Math.pow(t_vector[0], 2) + Math.pow(t_vector[1], 2));
    if (t_module > 0.1) {
        while (t_module > 0.1) {
            actual_position = positionment();
            //console.log(actual_position);
            //s = s + coords[1].toString() + "\n";
            var t_vector = [path[path.length - 1][0] - actual_position[0], path[path.length - 1][1] - actual_position[1]];
            var t_module = Math.sqrt(Math.pow(t_vector[0], 2) + Math.pow(t_vector[1], 2));
            //check if robor orientates or goes straight
            var orientationAngle = actual_position[2];
            var angleT = Math.atan2(t_vector[1], t_vector[0]);
            var angle = orientationAngle - angleT;
            if (angle > 2 * Math.PI) {
                angle -= 2 * Math.PI;
            }
            //console.log("angle", angle);
            if ((angle < 0.01 && angle > -0.01)) {
                /*
                if (previous_order != "up") {
                    stop();
                    sleep(100);
                }
                */
                up();
                //console.log("up");
                previous_order = "up";
            }
            else if (angle<=Math.PI && angle>0) {
                /*
                if (previous_order != "right") {
                    stop();
                    sleep(100);
                }
                */
                right();
                previous_order = "right";
            }
            else {
                /*
                if (previous_order != "left") {
                    stop();
                    sleep(100);
                }
                */
                left();
                previous_order = "left";
            }
        }
    }
    //textFile.writeFile('coords.txt', s);
};

var sleep = function (milliseconds) {
    var start = new Date().getTime();
    for (var i = 0; i < 1e7; i++) {
        if ((new Date().getTime() - start) > milliseconds) {
            break;
        }
    }
};




var path_following_openLoop = function (path) {
    var previous_order = "";
    var path_copy = path.slice(0, path.length);
    console.log("path", path_copy);
    console.log("pathCopy", path);
    var emergency_stop = emergency;
    if (path_copy.length == 0) {
        console.log("no path was found!");
    }
    else {
        var lastPoint = path_copy[path_copy.length - 1];
        while (path_copy.length > 1) {
            var t_vector = [path_copy[1][0] - path_copy[0][0], path_copy[1][1] - path_copy[0][1]];
            var t_module = Math.sqrt(Math.pow(t_vector[0], 2) + Math.pow(t_vector[1], 2));
            var timeUp = 1000 * t_module / 75; //speed = 0.75 m/s
            var contador = new Date().getTime();
            //console.log("contador", contador);
            for (var time_up = 0; time_up < timeUp;) {
                //go straight
                if (previous_order != "up") {
                    up();
                    previous_order = "up";
                    console.log("up!");
                }
                //console.log("up!", time_up);
                time_up = new Date().getTime() - contador;
                //console.log("time", time_up);
            }
            stop();
            stop();
            sleep(1000);
            if (path_copy[1].length == 3) {
                if (path_copy[4] == lastPoint) {
                    stop();
                    break;
                }
                //follow bezier curve. Check position and speed
                var bezierTime = new Date().getTime();
                for (var timer = 0; timer <= bezierRealTimes[0];) {
                    timer = new Date().getTime() - bezierTime;
                    var t_mod = 0.001 * timer / bezierRealTimes[0];
                    var speedToWrite = bezierSpeeds[0][Math.floor(100 * t_mod)];
                    var localRadius = bezierRadius[0][Math.floor(100 * t_mod)];
                    var motorSpeed = inverse_odometry(speedToWrite, localRadius);
                    writeSpeed(motorSpeed);   //uncomment to write to Arduino
                }
                path_copy.slice(0, 4);
                bezierSpeeds.shift();
                bezierRadius.shift();
                bezierRealTimes.shift();
            }
            else { //delete point from list
                stop();
                if (path_copy[1] == lastPoint) {
                    stop();
                    console.log("stop!");
                    break;
                }
                sleep(1000);
                //console.log("path", path_copy);
                var nextT = [path_copy[2][0] - path_copy[1][0], path_copy[2][1] - path_copy[1][1]];
                var angle = Math.atan2(nextT[0], nextT[1]) - Math.atan2(t_vector[0], t_vector[1]);
                sleep(1000);
                var giroTime = new Date().getTime();
                var timeForGiro = 1000 * Math.abs(angle) / 2; //omega = 2 rad/s
                //console.log("angle1", angle );
                for (var t = 0; t < timeForGiro;) { //turn for positioning
                    t = new Date().getTime() - giroTime;
                    if (angle <= Math.PI && angle > 0) {
                        if (previous_order != "left") {
                            left();
                            previous_order = "left";
                            console.log("left!");
                        }
                    }
                    else if (angle > -Math.PI && angle < 0) {
                        if (previous_order != "right") {
                            right();
                            previous_order = "right";
                            console.log("right!");
                        }
                    }
                }
                path_copy.shift();
                stop();
                sleep(1000);
            }
        }
        console.log("finished!");
        stop();
    }
};
