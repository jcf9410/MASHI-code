/*

2015/03/12: Adding REST operation to head motion

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
var serialPortROBOTIS = new SerialPortROBOTIS("COM5", {baudrate: 1000000}, false); // this is the openImmediately flag [default is true]
//var serialPortROBOTIS = new SerialPortROBOTIS("COM7", {baudrate: 57600}, false); // this is the openImmediately flag [default is true]

//Buffer to send values through serialport
var bufferHeadSize = 4;
var bufferBaseSize = 4;
bufferHead = new Buffer(bufferHeadSize);
bufferBase = new Buffer(bufferBaseSize);


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
			  status: 'Welcome to L\'Hospitalet #SCEWC15',
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
		bufferBase[1] = 20 + offset;
		bufferBase[2] = 20 + offset;
        baseCommand (bufferBase);
    },
    'down': function () {
        console.log('Reverse!');
        // baseCommand (VELO, -15 + offset, -15 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = -15 + offset;
		bufferBase[2] = -15 + offset;
        baseCommand (bufferBase);

    },
    'left': function () {
        console.log('Turn left!');
        // baseCommand (VELO, -15 + offset, 15 + offset);    
		bufferBase[0] = VELO;
		bufferBase[1] = -15 + offset;
		bufferBase[2] = 15 + offset;
        baseCommand (bufferBase);
		
	},
    'right': function () {
        console.log('Turn right!');
        // baseCommand (VELO, 15 + offset, -15 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = 15 + offset;
		bufferBase[2] = -15 + offset;
        baseCommand (bufferBase);
		
	},
    'space': function () {
        console.log('STOP!');
        // baseCommand (VELO, 0 + offset, 0 + offset);
		bufferBase[0] = VELO;
		bufferBase[1] = 0 + offset;
		bufferBase[2] = 0 + offset;
        baseCommand (bufferBase);
		
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
    }
    // 't': function () {
		// headRest();
    // },
    // 'g': function () {
		// headWake();
    // },
    // 'y': function () { // testing head motors
		// headTest();
    // }
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
        console.log('Arduino: ' + data);
		//console.log('Arduino->Server: ');
		//console.log(data);
    });
});
serialPortROBOTIS.open(function () {
    console.log('Server: serialportROTOBIS.open');
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
    } else {
      throw new Error("A socket with the name '"+name+"' does not exist");
    }
  }

};


