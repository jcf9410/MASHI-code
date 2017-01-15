'use strict';
/*
2015/04/20: Change base motion key events (w, a , s, d) to arrows and head motion events
2015/03/13: Resolving keypress events conflicts
2015/01/13: adding button event greetButton to make a sequence of head movements
*/
var DEBUG = 0XFF; // Debug Print.  Set to FF to debug
//base motion
//var STOP = 	0X00;
var EMER =	0X01;
var RARM =	0x02;
var VELO =	0X03;
var SMON =	0X04; //Social mode ON
var SMOF =	0X05; //Social mode OFF
var PIDC =	0X06;
var PIDA =	0X07;
var leftHand = 0;
var rightHand = 0;

var pitch = 0;
var yaw = 0;
var roll = 0;


var baseOffset = 100;
var baseVl, baseVr;
////////////////////

var pipes = new Array();
var numVideoStream = 1;
var data;
var sendChannel, receiveChannel, pcConstraint, dataConstraint;
//var dataChannelSend = document.querySelector('textarea#dataChannelSend');
var dataChannelSend = document.querySelector('input#dataChannelSend');
var dataChannelReceive = document.querySelector('textarea#dataChannelReceive');
var sctpSelect = document.querySelector('input#useSctp');
var rtpSelect = document.querySelector('input#useRtp');
var startButton = document.querySelector('button#startButton');
var sendTtsButton = document.querySelector('button#sendTtsButton');
var closeButton = document.querySelector('button#closeButton');
var msgTxt = document.getElementById('msgTxt');

var stopTimerButton = document.querySelector('button#stopTimerButton');


//var forwardButton = document.querySelector('button#forward');
//var leftButton = document.querySelector('button#left');
// var stopButton = document.querySelector('button#stop');
//var rightButton = document.querySelector('button#right');
//var reverseButton = document.querySelector('button#reverse');


//////////////////////////////////////////////////////////////////////
//HEAD
//////////////////////////////////////////////////////////////////////
/*var headRest = document.querySelector('button#headRest');
var headWake = document.querySelector('button#headWake');
*/

//var CabezaArribaButton = document.querySelector('button#CabezaArriba');
//var CabezaNormalButton = document.querySelector('button#CabezaNormal');
//var CabezaAbajoButton = document.querySelector('button#CabezaAbajo');

var carButton = document.querySelector('button#carButton');
var beatButton = document.querySelector('button#beatButton');
var loveButton = document.querySelector('button#loveButton');
/////////////////////////////////////////////////////////////////////
//var greetButton = document.querySelector('button#greetButton');
//var greetAllButton = document.querySelector('button#greetAllButton');
////////////////////////////////////////////////////////////////////


var happyButton = document.querySelector('button#happyButton');
var sadButton = document.querySelector('button#sadButton');
var angryButton = document.querySelector('button#angryButton');
var uncertainButton = document.querySelector('button#uncertainButton');
var neutralButton = document.querySelector('button#neutralButton');
var sleepyButton = document.querySelector('button#sleepyButton');
var intensitySlider = document.querySelector('input#intensity');
var intensityLabel = document.querySelector('label#intensityLabel');


var localVideoPanel = document.querySelector('div#localVideoPanel');
var localVideo = document.querySelector('video#localVideo');
var robotIcon = document.querySelector('object#robotIcon');
var menuMessage = document.querySelector('p#menuMessage');
var controlCheckbox = document.querySelector('input#controlCheckbox');
var modeSelector = document.querySelector('select#modeSelector');
var selectRoboticonMode = document.querySelector('select#roboticonModeSelect');


////////////////////////////
// VOICE VARIABLES
////////////////////////////
var ttsCheckbox = document.querySelector('input#ttsCheckbox');


var localVideo = document.querySelector('#localVideo');
//var vid1 = document.querySelector('#vid1');
var remoteVideoFrontal = document.querySelector('#remoteVideoFrontal');
var remoteVideoOmni = document.querySelector('#remoteVideoOmni');


var audioTracks;
var webAudio;
var filteredStream;
var intensity;

////////////////////////////////////////////////////
// Events
////////////////////////////////////////////////////
//startButton.onclick = createConnection;
sendTtsButton.onclick = sendTtsData;
//RobotIcon button
//controlCheckbox.onclick = toggleControl;
happyButton.onclick = sendHappy;
sadButton.onclick = sendSad;
angryButton.onclick = sendAngry;
uncertainButton.onclick = sendUncertain;
neutralButton.onclick = sendNeutral;
sleepyButton.onclick = sendSleepy;
selectRoboticonMode.onchange = changeRoboticonDisplayMode;
modeSelector.onchange = changeMode;
intensitySlider.onchange = updateIntensity;

stopTimerButton.onclick = stopTimer;

/////////////////////////////////////////////////
/////////////////////////////////////////////////
// Expo
/////////////////////////////////////////////////
/////////////////////////////////////////////////
modeInicio.onchange = changeInicio;
modeDesarrollo.onchange = changeDesarrollo;
//modeCuestionario.onchange = changeCuestionario;
modeFrase.onchange = changeFrase;



$(document).keydown(handleKeyControl);
//////////////////////////////////////////////////
// VOICE CONTROL EVENTS
//////////////////////////////////////////////////
ttsCheckbox.onclick = toggleTts;
dataChannelSend.onkeypress = handleSendKeyPress;

//keyControl.onkeypress = handleKeyControl;

//////////////////////////////////////////////////
// SOUNDS
//////////////////////////////////////////////////

var playSoundButton2 = document.querySelector('button#playSoundButton');

var listSounds = [];
var context;
var effect;
var soundBuffer;

var sound01Buffer = null;
var sound02Buffer = null;
var sound03Buffer = null;
var sound04Buffer = null;
var sound05Buffer = null;
var sound06Buffer = null;

// sound01Button.onclick = playSound01;
// sound02Button.onclick = playSound02;
// sound03Button.onclick = playSound03;
// sound04Button.onclick = playSound04;
// sound05Button.onclick = playSound05;
// sound06Button.onclick = playSound06;

// playSoundButton2.onclick = playSoundButton;
// stopSoundButton.onclick = stopSound;

loadSounds.onclick = requestListSounds;



// Get the voice select element.
var soundSelect = document.getElementById('soundList');

///////////////////////////////////////////
//HEAD EVENTS
///////////////////////////////////////////
headRest.onclick = sendHeadRest; 
headWake.onclick = sendHeadWake;
// testHeadButton.onclick = playTestHead;
//greetAllButton.onclick = playGreetAll;
///////////////////////////////////////////
// DIALOGUE
///////////////////////////////////////////
//dialogueButton.onclick = playDialogue;
///////////////////////////////////////////

///////////////////////////////////////////
//ROBOT CONFIGURATION 
///////////////////////////////////////////
baseStop.onclick = baseStop; 
baseRearm.onclick = baseRearm;


///////////////////////////////////////////
// VOLUMEN  
///////////////////////////////////////////
//var robotVolInput = document.getElementById('robotVol'); //robotVolInput -> robotVol
//var configRobotVolButton = document.getElementById('configRobotVol');

///////////////////////////////////////////
// ROBOT VELOCITY GAIN  
///////////////////////////////////////////
var robotVelocityGain = document.getElementById('robotVelocityGain');


///////////////////////////////////////////
// PID MODE  
///////////////////////////////////////////
var pidConservative = document.getElementById('pidConservative');
var pidAggresive = document.getElementById('pidAggresive');
pidConservative.onclick = sendPidConservative;
pidAggresive.onclick = sendPidAggresive;


///////////////////////////////////////////
// flagEvent 
///////////////////////////////////////////
var flagEvent = document.getElementById('flagEvent');
flagEvent.onclick = changeFlagEvent;

///////////////////////////////////////////
// SOCIAL MODE  
///////////////////////////////////////////
var socialModeOn = document.getElementById('socialModeOn');
var socialModeOff = document.getElementById('socialModeOff');
socialModeOn.onclick = sendSocialModeOn;
socialModeOff.onclick = sendSocialModeOff;

var user='dennys';
var isChannelReady;
var isInitiator = false;
var isStarted = false;
var localStream;
var pc;
var remoteStream;
var turnReady;

var constraints;
var pc_config = { 'iceServers': [{ 'url': 'stun:stun.l.google.com:19302' }] };
/*var pc_config = {
  'iceServers': [
    {
      'url': 'stun:stun.l.google.com:19302'
    },
    {
      'url': 'turn:192.158.29.39:3478?transport=udp',
      'credential': 'JZEOEt2V3Qb0y27GRntt2u2PAYA=',
      'username': '28224511:1379330808'
    },
    {
      'url': 'turn:192.158.29.39:3478?transport=tcp',
      'credential': 'JZEOEt2V3Qb0y27GRntt2u2PAYA=',
      'username': '28224511:1379330808'
    }
  ]
}*/

//var pc_config = { 'iceServers': [{ 'url': 'http://signaling.simplewebrtc.com:8888' }] };

var pc_constraints = {'optional': [{'DtlsSrtpKeyAgreement': true}]};

// Set up audio and video regardless of what devices are present.
var sdpConstraints = {'mandatory': {
  'OfferToReceiveAudio':true,
  'OfferToReceiveVideo':true }};

/////////////////////////////////////////////

/*var room = location.pathname.substring(1);
if (room === '') {
  room = 'robotRoom'; //prompt('Enter room name:');
} else {
    //

}
*/


///////////////////////////////////
// Tomo las fuentes de video que existen en la compu. Las necesito para añadir las camaras fuente (frente y omnidireccional) en el robot
///////////////////////////////////
var videoSources = [];
MediaStreamTrack.getSources(function (media_sources) {
    //console.log('media sources: ' + media_sources);
    //alert('media_sources : '+media_sources);
    media_sources.forEach(function (media_source) {
        if (media_source.kind === 'video') {
            //console.log("media_source..." + media_source.label);
            videoSources.push(media_source);
        }
    });
    //console.log("llama a funcion getMediaSource...");
    //getMediaSource(videoSources);
    //console.log('video sources: ' + videoSources[2].label);
});

//////////////////////////////////////////////////////////////////////////////

var socket = io.connect();
var room = 'robotRoom';
var evento = '';


if (room !== '') {
    socket.emit('create or join', room);
}

socket.on('created', function (room){
  // Aqui entra el iniciador de la conversacion ei robot
  //console.log('socket.on created');
  //isInitiator = true;
  user='dennys';
  console.log('Created: ' + user + ' isInitiator: ' + isInitiator);
    ////////////////////////////////////////////////
  // getUserMedia para

  //basic constraints
  //constraints = { video: true, audio: true };
  //QVGA resultante : 176x144
  constraints = { audio: true, video: { mandatory: { maxWidth: 320, maxHeight: 180} } };
  //VGA video Stream resultante: 352x288
  //constraints = { audio: true, video: { mandatory: { maxWidth: 640, maxHeight: 360, googCpuOveruseDetection: true, googLeakyBucket: true  } } };
  //HD video 1280x720
  //constraints = { audio: true, video: { mandatory: { minWidth: 1280, minHeight: 720, googCpuOveruseDetection: true, googLeakyBucket: true } } };

  console.log(user + ' getUserMedia, constraints', constraints);
  //getUserMedia(constraints, handleUserMedia, handleUserMediaError);
  	navigator.mediaDevices.getUserMedia(constraints).
    then(handleUserMedia).catch(handleUserMediaError);


});


socket.on('join', function (room){
  // aqui entra el robot
  console.log(user + ' join ' + room + '!');
  isChannelReady = true;
});

socket.on('joined', function (room){
    console.log(user + ' joined ' + room);
    isChannelReady = true;
    //console.log('isChannelReady ' + isChannelReady);

    //QVGA resultante : 176x144
    constraints = { audio: true, video: { mandatory: { maxWidth: 320, maxHeight: 180} } };
    //constraints = { audio: true, video: false};

    //getUserMedia(constraints, handleUserMedia, handleUserMediaError);
		navigator.mediaDevices.getUserMedia(constraints).
    then(handleUserMedia).catch(handleUserMediaError);

    //console.log(user + ' getUserMedia con constraints', constraints);

});

socket.on('full', function (room){
  console.log('No hay cama pa tanta gente, el cuarto ' + room + ' esta lleno!');
});


socket.on('log', function (array){
  console.log.apply(console, array);
});


socket.on('message', function (message){
  console.log(user + ' recibe mensaje de server:', message);

  if (message === 'got user media') {
    console.log('entro a message->got user media');
    maybeStart();
  }
  else if (message.type === 'offer') {
    console.log(user + 'recibe mensaje de oferta');
    if (!isInitiator && !isStarted) {
      maybeStart();
    }
    pc.setRemoteDescription(new RTCSessionDescription(message));
    doAnswer();
  }
  else if (message.type === 'answer' && isStarted) {
    console.log(user + ' recibe mensaje de respuesta');
    pc.setRemoteDescription(new RTCSessionDescription(message));
  }
  else if (message.type === 'candidate' && isStarted) {
    console.log(user + ' recibe mensaje de candidate');
    var candidate = new RTCIceCandidate({
      sdpMLineIndex: message.label,
      candidate: message.candidate
    });
    console.log(user + ' AÑADE ICE CANDIDATE: ' + message.candidate);
    pc.addIceCandidate(candidate);
  } else if (message === 'bye' && isStarted) {
    handleRemoteHangup();
  }
});
socket.on('responseListSounds', function (array){
	console.log('responseListSounds4...');
	listSounds = JSON.parse(array);
	console.log(listSounds);
	// Execute loadSoundsList.
	loadSoundsList();
	
  });

socket.on('event', function (data){
	console.log(data);
	msgTxt.value = 'event: ' + data;
	if (data == 'RHAND' && flagEvent.checked) {
		playNavidad();
	}
});
 
  // // Fetch the list of sounds and populate the voice options.
function loadSoundsList() {
  // Fetch the available sounds.
  //var voices = speechSynthesis.getVoices();
  // Loop through each of the sounds.
  listSounds.forEach(function(sound, i) {
    // Create a new option element.
    var option = document.createElement('option');
    
    // Set the options value and text.
    option.value = sound;
    option.innerHTML = sound;
    // Add the option to the voice selector.
    soundSelect.appendChild(option);
  });
}


  
////////////////////////////////////////////////

function sendMessage(message){
	console.log(user + ' envia mensaje al server: ', message);
  // if (typeof message === 'object') {
  //   message = JSON.stringify(message);
  // }
  socket.emit('message', message);
}



function handleUserMedia(stream) {
  console.log(user + ' handleUserMedia');
  localVideo.src = window.URL.createObjectURL(stream);
  //localStream = stream;
  audioTracks = stream.getAudioTracks();
  if (audioTracks.length == 1) {
      console.log('HANDLE USER MEDIA AUDIO TRACK == 1');
      filteredStream = applyFilter(stream);
      localStream = stream;
    } else {
      alert('The media stream contains an invalid amount of audio tracks.');
      stream.stop();
    }

  sendMessage('got user media');
  console.log('handleUserMedia: '+ user+ ' isInitiator: ' + isInitiator);
  if (isInitiator) {
    maybeStart();
  }
}

function handleUserMediaError(error){
  console.log('getUserMedia error: ', error);
}

//var constraints = {video: false };


//var constraints = { video: true };
//getUserMedia(constraints, handleUserMedia, handleUserMediaError);
//console.log('Getting user media with constraints', constraints);

/*dp
if (location.hostname != "localhost") {
  requestTurn('https://computeengineondemand.appspot.com/turn?username=41784574&key=4080218913');
}
*/

function maybeStart() {
  console.log('maybeStart ' + user);
  console.log('!isStarted=' + !isStarted + ' localStream: ' + localStream + ' isChannelReady ' + isChannelReady);
  if (!isStarted && typeof localStream != 'undefined' && isChannelReady) {
    console.log('maybeStart ' + user + ' entra al if');
    createPeerConnection();
    createDataChannel();

    pc.ondatachannel = receiveChannelCallback;

    sendChannel.onopen = onSendChannelStateChange;
    sendChannel.onclose = onSendChannelStateChange;
    //dp audio
	//pc.addStream(loveStream);
    pc.addStream(filteredStream);
    pc.addStream(localStream);

    isStarted = true;
    console.log('maybeStart ' + user + ' isInitiator ', isInitiator);
    if (isInitiator) {
      doCall();
    }
  }
}

window.onbeforeunload = function(e){
	sendMessage('bye');


}

/////////////////////////////////////////////////////////

function createPeerConnection() {
  try {
    pc = new RTCPeerConnection(pc_config);
    pc.onicecandidate = handleIceCandidate;
    pc.onaddstream = handleRemoteStreamAdded;
    pc.onremovestream = handleRemoteStreamRemoved;
    console.log(user + ' RTCPeerConnectionnection creada');
  } catch (e) {
    console.log('Failed to create PeerConnection, exception: ' + e.message);
    alert('Cannot create RTCPeerConnection object.');
      return;
  }
}

function createDataChannel(){
  dataChannelSend.placeholder = '';
  dataConstraint = null;
  try {
    // Data Channel api supported from Chrome M25.
    // You might need to start chrome with  --enable-data-channels flag.
    sendChannel = pc.createDataChannel('sendDataChannel', dataConstraint);
    trace(user + ' crea send data channel');
  } catch (e) {
    alert('Failed to create data channel. ' +
          'You need Chrome M25 or later with --enable-data-channels flag');
    trace('Create Data channel failed with exception: ' + e.message);
  }
}



function handleIceCandidate(event) {
  console.log(user + ' handleIceCandidate: ', event);
  if (event.candidate) {
    sendMessage({
      type: 'candidate',
      label: event.candidate.sdpMLineIndex,
      id: event.candidate.sdpMid,
      candidate: event.candidate.candidate});
  } else {
    console.log('Final de candidatos.');
  }
}


function handleCreateOfferError(event){
  console.log('createOffer() error: ', e);
}

function handleCreateAnswerError(event){
  console.log('createAnswer() error: ', e);
}

function doCall() {
  console.log(user + ' envia una oferta.');
  pc.createOffer(setLocalAndSendMessage, handleCreateOfferError);
}

function doAnswer() {
  console.log(user + 'Envia una respuesta.');
  pc.createAnswer(setLocalAndSendMessage, handleCreateAnswerError, sdpConstraints);
}

function setLocalAndSendMessage(sessionDescription) {
  // Set Opus as the preferred codec in SDP if Opus is present.
  sessionDescription.sdp = preferOpus(sessionDescription.sdp);
  pc.setLocalDescription(sessionDescription);
  console.log(user + ' setLocalAndSendMessage sessionDescription: ' , sessionDescription);
  sendMessage(sessionDescription);
}

function requestTurn(turn_url) {
  console.log('Entro a requestTurn');
  var turnExists = false;
  for (var i in pc_config.iceServers) {
    console.log('Entro a var i in pc_config.iceServers: ' + pc_config.iceServers);
    if (pc_config.iceServers[i].url.substr(0, 5) === 'turn:') {
      console.log('entro a turnExists = true');
      turnExists = true;
      turnReady = true;
      break;
    }
  }
  if (!turnExists) {
    console.log('Getting TURN server from ', turn_url);
    // No TURN server. Get one from computeengineondemand.appspot.com:
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function(){
      if (xhr.readyState === 4 && xhr.status === 200) {
        var turnServer = JSON.parse(xhr.responseText);
      	console.log('Got TURN server: ', turnServer);
        pc_config.iceServers.push({
          'url': 'turn:' + turnServer.username + '@' + turnServer.turn,
          'credential': turnServer.password
        });
        turnReady = true;
      }
    };
    xhr.open('GET', turn_url, true);
    xhr.send();
  }
}

function handleRemoteStreamAdded(event) {
	console.log(' handleRemoteStreamAdded...');
  if (numVideoStream==1){
    console.log(user + 'FRONTAL: ' + event.stream.id);
    remoteVideoFrontal.src = window.URL.createObjectURL(event.stream);
    remoteStream = event.stream;
  }
  else if (numVideoStream==2){
    console.log(user + 'OMNI: ' + event.stream.id);
    remoteVideoOmni.src = window.URL.createObjectURL(event.stream);
    remoteStream = event.stream;
  }
  numVideoStream=numVideoStream+1;

}


/*function handleRemoteStreamAdded(event) {
  console.log(user + ' Remote stream added.');
  remoteVideo.src = window.URL.createObjectURL(event.stream);
  remoteStream = event.stream;
}
*/
function handleRemoteStreamRemoved(event) {
  console.log('Remote stream removed. Event: ', event);
}

function hangup() {
  console.log('Hanging up.');
  stop();
  sendMessage('bye');
}

function handleRemoteHangup() {
  console.log('Session terminated.');
   stop();
   //isInitiator = false;
}

function stop() {
  isStarted = false;
  // isAudioMuted = false;
  // isVideoMuted = false;
  pc.close();
  pc = null;
}


/*  function stop() {
    webAudio.stop();

    // pc1.close();
    // pc2.close();
    // pc1 = null;
    // pc2 = null;

     pc.close();
     pc = null;


    buttonStart.enabled = true;
    buttonStop.enabled = false;
    localStream.stop();
  }

*/

///////////////////////////////////////////
// Set Opus as the default audio codec if it's present.
///////////////////////////////////////////
function preferOpus(sdp) {
  var sdpLines = sdp.split('\r\n');
  var mLineIndex;
  // Search for m line.
  for (var i = 0; i < sdpLines.length; i++) {
      if (sdpLines[i].search('m=audio') !== -1) {
        mLineIndex = i;
        break;
      }
  }
  if (mLineIndex === null) {
    return sdp;
  }

  // If Opus is available, set it as the default in m line.
  for (i = 0; i < sdpLines.length; i++) {
    if (sdpLines[i].search('opus/48000') !== -1) {
      var opusPayload = extractSdp(sdpLines[i], /:(\d+) opus\/48000/i);
      if (opusPayload) {
        sdpLines[mLineIndex] = setDefaultCodec(sdpLines[mLineIndex], opusPayload);
      }
      break;
    }
  }

  // Remove CN in m line and sdp.
  sdpLines = removeCN(sdpLines, mLineIndex);

  sdp = sdpLines.join('\r\n');
  return sdp;
}

function extractSdp(sdpLine, pattern) {
  var result = sdpLine.match(pattern);
  return result && result.length === 2 ? result[1] : null;
}

// Set the selected codec to the first in m line.
function setDefaultCodec(mLine, payload) {
  var elements = mLine.split(' ');
  var newLine = [];
  var index = 0;
  for (var i = 0; i < elements.length; i++) {
    if (index === 3) { // Format of media starts from the fourth.
      newLine[index++] = payload; // Put target payload to the first.
    }
    if (elements[i] !== payload) {
      newLine[index++] = elements[i];
    }
  }
  return newLine.join(' ');
}

// Strip CN from sdp before CN constraints is ready.
function removeCN(sdpLines, mLineIndex) {
  var mLineElements = sdpLines[mLineIndex].split(' ');
  // Scan from end for the convenience of removing an item.
  for (var i = sdpLines.length-1; i >= 0; i--) {
    var payload = extractSdp(sdpLines[i], /a=rtpmap:(\d+) CN\/\d+/i);
    if (payload) {
      var cnPos = mLineElements.indexOf(payload);
      if (cnPos !== -1) {
        // Remove CN payload from m line.
        mLineElements.splice(cnPos, 1);
      }
      // Remove CN line in sdp
      sdpLines.splice(i, 1);
    }
  }

  sdpLines[mLineIndex] = mLineElements.join(' ');
  return sdpLines;
}

///////////////////////////////////////////
// Funciones datachannel
///////////////////////////////////////////
function receiveChannelCallback(event) {
  trace('Receive Channel Callback');
  receiveChannel = event.channel;
  receiveChannel.onmessage = onReceiveMessageCallback;
  receiveChannel.onopen = onReceiveChannelStateChange;
  receiveChannel.onclose = onReceiveChannelStateChange;
}


function onReceiveMessageCallback(event) {
  trace(user + ' recibe dato: ' + event.data);
  dataChannelReceive.value = event.data;
  // aqui viene la funcion para enviar el dato al servidor
  // Hay dos tipos de mensajes: ACTUAR y HABLAR
  if (dataChannelReceive.value.substr(0, 7) === 'HABLAR:'){
    console.log(user + ' HABLAR ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    //speak(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
    //window.speechSynthesis.speak(msg);
    //speechSynthesis.speak(SpeechSynthesisUtterance('Hello World'));
    var msg = new SpeechSynthesisUtterance(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
    window.speechSynthesis.speak(msg);
    //window.speechSynthesis.speak(SpeechSynthesisUtterance(dataChannelReceive.value.substr(7,dataChannelReceive.value.length)));

  }

}






function onSendChannelStateChange() {
  var readyState = sendChannel.readyState;
  trace('Send channel state is: ' + readyState);
  if (readyState == 'open') {
  	//alert("Data channel OPEN!");
    //dataChannelSend.disabled = false;
    //dataChannelSend.focus();
    //sendButton.disabled = false;
    //closeButton.disabled = false;
  } else {
	//alert("Data channel not opened!");
    dataChannelSend.disabled = true;
    sendButton.disabled = true;
    //closeButton.disabled = true;
  }
}


function onReceiveChannelStateChange() {
  var readyState = receiveChannel.readyState;
  trace('Receive channel state readyState is: ' + readyState);
}


//funciones para enviar datos al robot mediante el datachannel
function handleSendKeyPress(event) {
  console.log ('handleSendKeyPress...' + event.keyCode + ' '+ event.which);
  var key=event.keyCode || event.which;
  if (key==13 && ttsCheckbox.checked){
    console.log('sendTtsData..');
    sendTtsData();

  }
}

function handleKeyControl(event) {
    console.log('handleKeyControl....');
	if(!ttsCheckbox.checked )
	{
		if(DEBUG){
			console.log ('handleKeyControl...' + event.keyCode + ' event.which: '+ event.which);
		}
	  var key = event.keyCode;
	  switch(key) {
		// Base motions
		case 49://EMERGENGY STOP base motor. Keyboard No. 1
			data = 'ACTUAR:'+1+';'+EMER;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		
		case 50://REARM base motor Keyboard No. 2
			data = 'ACTUAR:'+1+';'+RARM;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		
		case 32: //space bar: STOP
			//data = 'ACTUAR:1;100;100';
			data = 'ACTUAR:'+3+';'+VELO+';'+100+';'+100;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
	//		keyControl.value = null;
			break;
		case 38: //key "UP ARROW": forward
			//data = 'ACTUAR:1;140;140';
			baseVl = 100 * robotVelocityGain.value + baseOffset;
			baseVr = 100 * robotVelocityGain.value + baseOffset;
			data = 'ACTUAR:'+3+';'+VELO+';'+baseVl+';'+baseVr;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		case 40://key "DOWN ARROW": backward
			//data = 'ACTUAR:1;60;60';
			baseVl = -100 * robotVelocityGain.value + baseOffset;
			baseVr = -100 * robotVelocityGain.value + baseOffset;
			data = 'ACTUAR:'+3+';'+VELO+';'+baseVl+';'+baseVr;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		case 37://key "LEFT ARROW": left
//			data = 'ACTUAR:1;60;140';
			baseVl = -100 * robotVelocityGain.value * 0.5 + baseOffset;
			baseVr = 100 * robotVelocityGain.value * 0.5 + baseOffset;
			data = 'ACTUAR:'+3+';'+VELO+';'+baseVl+';'+baseVr;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		case 39://key "RIGHT ARROW": right
//			data = 'ACTUAR:1;140;60';
			baseVl = 100 * robotVelocityGain.value * 0.5 + baseOffset;
			baseVr = -100 * robotVelocityGain.value * 0.5 + baseOffset;
			data = 'ACTUAR:'+3+';'+VELO+';'+baseVl+';'+baseVr;
			trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;

		/////////////////////////////////////////////
		// Head motions
		/////////////////////////////////////////////
		case 83://key "s": zero head
			data = 'HEADMO:50;50;50';
			pitch = 50;
			yaw = 50;
			roll = 50;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 87:// key "w": pitchup
			//data = 'HEADMO:70;50;50';
			pitch = 70;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 88:// key "x": pitchdown
			//data = 'HEADMO:0;50;50';
			pitch = 0;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 65://key "a": yawleft
			//data = 'HEADMO:50;100;50';
			yaw = 100;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 68://key "d": yawright
			//data = 'HEADMO:50;0;50';
			yaw = 0;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 81://key "q": rollLeft pitch up
			//data = 'HEADMO:40;50;100';
			pitch = 40;
			yaw = 50;
			roll = 100;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 69://key "e": rollRight pitch up
			//data = 'HEADMO:40;50;0';
			pitch = 40;
			yaw = 50;
			roll = 0;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 90://key "z": rollLeft pitch down
			//data = 'HEADMO:0;50;100';
			roll = 100;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		case 67://key "c": rollRight pitch down
			//data = 'HEADMO:0;50;0';
			roll = 0;
			data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
			sendChannel.send(data);
			break;
		///////////////////////////////	
		//ARMS MOVEMENTS
		///////////////////////////////	
		//LEFT
		case 82://key "r"
			//arms up
			data = 'ARMSMO:100;100;100';
			// trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		case 70://key "f"
			//left arm up
			leftHand = 100;
			data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
			sendChannel.send(data);
			break;
		case 86://key "v" left arm down
			leftHand = 0;
			data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
			// trace(user + ' envia dato: ' + data);
			sendChannel.send(data);
			break;
		case 71://key "g" left shake
			shakeLeft();
			break;
		
		//RIGHT
		case 85://key "u"
			//arms up
			data = 'ARMSMO:100;100;100';
			sendChannel.send(data);
			break;

		case 74://key "j"
			rightHand = 100;
			data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
			sendChannel.send(data);
			break;
			
		case 77://key "m" down
			rightHand = 0; //
			data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
			sendChannel.send(data);
			break;
		case 72://key "h" shake
			shakeRight();
			break;
		///////////////////////////////
		//SHOULDER MOVEMENTS
		///////////////////////////////	
		case 79://key "o"
			shoulderUp();
			break;
		case 76://key "l" shake
			shoulderDown();
			break;
			
			
		}
	}
  
}

///////////////////////////////////////////////
// VOICE
///////////////////////////////////////////////
 function toggleTts()
 {
		if(ttsCheckbox.checked)
		{
  		dataChannelSend.disabled = false;
  		sendTtsButton.disabled = false;
  		sendTtsButton.style.background = "#1E90FF";
  		sendTtsButton.style.borderBottom = "#7d7d7d";
		}
		else
		{
  		dataChannelSend.disabled = true;
  		sendTtsButton.disabled = true;
  		sendTtsButton.style.background = "#E3E3E3";
  		sendTtsButton.style.borderBottom = "#E3E3E3";
		}
 }


function sendTtsData() {
  var data = 'HABLAR:'+dataChannelSend.value;
  sendChannel.send(data);
  trace(user + ' sends data: ' + data);
  dataChannelSend.value = null;
}


////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
//AUDIO INPUT SOUNDS FUNCTIONS
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
var robotVol = {};
robotVol.changeVolume = function(element) {
  var volume = element.value;
  var fraction = parseInt(element.value) / parseInt(element.max);
  // Let's use an x*x curve (x-squared) since simple linear (x) does not
  // sound as good.
//  this.gainNode.gain.value = fraction * fraction;
  sendChannel.send('SROVOL:' + fraction * fraction);
  
  
};


var VolumeSample = {};

// Gain node needs to be mutated by volume control.
VolumeSample.gainNode = null;

VolumeSample.changeVolume = function(element) {
	console.log('changeVolume..');
  var volume = element.value;
  var fraction = parseInt(element.value) / parseInt(element.max);
  // Let's use an x*x curve (x-squared) since simple linear (x) does not
  // sound as good.
  this.gainNode.gain.value = fraction * fraction;
};

window.addEventListener('load', init, false);

function init() {
  try {
	console.log ('load ini');
    // Fix up for prefixing
	numVideoStream = 1;
    window.AudioContext = window.AudioContext||window.webkitAudioContext;
    context = new AudioContext();
	window.filter = context.createBiquadFilter();
    //window.filter.type = 0; // Low-pass filter. See BiquadFilterNode docs
    //window.filter.frequency.value = 440; 
    window.filter.type = filter.HIGHPASS;
    window.filter.frequency.value = 1500;
	 }
  catch(e) {
    alert('Web Audio API is not supported in this browser');
  }
}



// Fix up prefixing
/*window.AudioContext = window.AudioContext || window.webkitAudioContext;
var context = new AudioContext();
*/


function loadSoundFile(url) {
  console.log('loadSoundFile..'+url);
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      soundBuffer = buffer;
    }, onError);
  }
  request.send();
}



function loadSound01(url) {
  console.log('loadSound01..'+url);
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      sound01Buffer = buffer;
    }, onError);
  }
  request.send();
}



/*
function playSound(buffer) {
  console.log('entro playSound..');
  var source = context.createBufferSource(); // creates a sound source
  source.buffer = buffer;                    // tell the source which sound to play
  source.connect(context.destination);       // connect the source to the context's destination (the speakers)
  source.start(0);                           // play the source now
                                             // note: on older systems, may have to use deprecated noteOn(time);
}
*/

function onError() {
  console.log('entro a onError');
}


function playSoundFile(url) {
	
  console.log('playSoundFile..'+url);
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      soundBuffer = buffer;
    }, onError);
  }
  request.send();
	
	
//  webAudio.addEffect();
  //playSound (dogBarkingBuffer);
  console.log ('playSoundFile..');
	effect = context.createBufferSource();
    effect.buffer = soundBuffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      console.log ('ENTRO A WINDOW.PEER..');
      effect.connect(window.peer);
      effect.start(0);
    }
}


function playSound01() {
//  webAudio.addEffect();
  //playSound (dogBarkingBuffer);
  console.log ('playSound01..');
    effect = context.createBufferSource();
    effect.buffer = sound01Buffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      console.log ('ENTRO A WINDOW.PEER..');
      effect.connect(window.peer);
      effect.start(0);
    }
}

function getData(url) {
	console.log('getData..'+url);
	effect = context.createBufferSource();
	var request = new XMLHttpRequest();
	request.open('GET', url, true);
	request.responseType = 'arraybuffer';
	// Decode asynchronously
	request.onload = function() {
	  console.log('request.onload..');
		context.decodeAudioData(request.response, function(buffer) 
		{
			console.log('decodeAudioData...'+url);
			effect.buffer = buffer;
		}, onError);
  }
  request.send();
}


VolumeSample.play = function() {
  if (!context.createGain)
    context.createGain = context.createGainNode;
  this.gainNode = context.createGain();


  if (soundSelect.value) {
	var audioFile = '/audio/'+ soundSelect.value;  
	//loadSoundFile ('/audio/'+soundSelect.value);
	getData(audioFile);
	// Create a gain node.
	//var gainNode = context.createGain();
	// Connect the source to the gain node.
	//effect.connect(gainNode);
	// Connect the gain node to the destination.
	//gainNode.connect(context.destination);
	//gainNode.start(0);
//	effect.connect(window.peer);
//	effect.start(0);

  // Connect source to a gain node
  effect.connect(this.gainNode);
  // Connect gain node to destination
  //this.gainNode.connect(context.destination);
  this.gainNode.connect(window.peer);
  // Start playback in a loop
  effect.loop = false;
  if (!effect.start)
    effect.start = effect.noteOn;
  effect.start(0);
  this.effect = effect;




	}



};

VolumeSample.stop = function() {
  if (!this.effect.stop)
    this.effect.stop = effect.noteOff;
  this.effect.stop(0);
};

VolumeSample.toggle = function() {
  this.playing ? this.stop() : this.play();
  this.playing = !this.playing;
};

function playSoundButton() {
  console.log ('playSoundButton..');
  //var effect = context.createBufferSource();
  if (soundSelect.value) {
	var audioFile = '/audio/'+ soundSelect.value;  
	//loadSoundFile ('/audio/'+soundSelect.value);
	getData(audioFile);
	// Create a gain node.
	//var gainNode = context.createGain();
	// Connect the source to the gain node.
	//effect.connect(gainNode);
	// Connect the gain node to the destination.
	//gainNode.connect(context.destination);
	//gainNode.start(0);
	effect.connect(window.peer);
	effect.start(0);
  }
}


function stopSound() {
  console.log ('stopSound..');
  //var effect = context.createBufferSource();
    if (window.peer) {
      console.log ('stopSound..');
      //effect.connect(window.peer);
      effect.stop(0);
    }
}

function requestListSounds() {
  console.log ('Operator: requestListSounds');
  socket.emit('requestListSounds', "hola");
}


function shoulderUp() {
  console.log ('shoulderUp..');
  socket.emit('SHLDUP', "hola");
}
function shoulderDown() {
  console.log ('shoulderDown..');
  socket.emit('SHLDDO', "hola");
}


///////////////////////////////////////////////////////////
// SHAKE LEFT HAND
///////////////////////////////////////////////////////////
function shakeLeft() {
	//console.log ('ENTRO playTestHead..');
	var delay=0;//
	var counter = 0;
	while (counter<4)
	{
	counter = counter + 1;
	//t0
	delay = delay + 200;
    setTimeout(function(){
	leftHand = 100;
	data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t1
	delay = delay + 200;
    setTimeout(function(){
	leftHand = 90;
	data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	}
  }

///////////////////////////////////////////////////////////
// SHAKE RIGHT HAND
///////////////////////////////////////////////////////////
function shakeRight() {
	//console.log ('ENTRO playTestHead..');
	var delay=0;//
	var counter = 0;
	while (counter<4)
	{
	counter = counter + 1;
	//t0
	delay = delay + 200;
    setTimeout(function(){
	rightHand = 100;
	data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t1
	delay = delay + 200;
    setTimeout(function(){
	rightHand = 90;
	data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	}
  }

///////////////////////////////////////////////////////////
// TESTING HEAD MOVEMENTS
///////////////////////////////////////////////////////////
function playTestHead() {
	//console.log ('ENTRO playTestHead..');
	var delay=0;//1 seconds
	var counter = 0;
	while (counter<1)
	{
	counter = counter + 1;
	//t0
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;0;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t1
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:100;0;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t2
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:100;100;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t3
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;100;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t4
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;0;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;100;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:100;100;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:100;0;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;0;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t
	delay = delay + 2000;
    setTimeout(function(){
	data = 'HEADMO:0;100;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	}
  }

function playGreetAll() {
	//console.log ('ENTRO playGreet..');
	var delay=0;//1 seconds
	//t3 cabeza zero
    setTimeout(function(){
	data = 'HEADMO:50;50;50';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	//t3 cabeza zero
	delay = delay + 1500;
    setTimeout(function(){
	data = 'HEADMO:50;100;0';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	delay = delay + 1500;

    setTimeout(function(){
	data = 'HEADMO:50;0;100';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 
	delay = delay + 1500;

    setTimeout(function(){
	data = 'HEADMO:50;50;50';
	sendChannel.send(data);
		//your code to be executed after 1 seconds
    },delay); 

	}

function stopTimer() {
	clearInterval(myVar01);
	clearInterval(myVar02);
	clearInterval(myVar03);
	clearInterval(myVar04);
	clearInterval(myVar05);
}

function myTimer01() {
		var delay = 0;
		setTimeout(function(){
		//sendHappy;sendSad;sendAngry;sendUncertain;sendNeutral;sendSleepy;
		intensity = 0.0;
		sendHappy();
		pitch = 30;
		yaw = 0;
		roll = 0;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 50;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
		//
		//t3 cabeza zero
		delay = delay + 1000;
		setTimeout(function(){
		intensity = 0.4;
		sendHappy();
		pitch = 0;
		yaw = 100;
		roll = 100;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 50;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
}

function myTimer02() {
		var delay = 0;
		setTimeout(function(){
		//sendHappy;sendSad;sendAngry;sendUncertain;sendNeutral;sendSleepy;
		intensity = 0.0;
		sendHappy();
		pitch = 0;
		yaw = 0;
		roll = 0;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
		//
		//t3 cabeza zero
		delay = delay + 1000;
		setTimeout(function(){
		intensity = 0.4;
		sendHappy();
		pitch = 0;
		yaw = 100;
		roll = 100;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 50;
		rightHand = 50;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
}

function myTimer03() {
		var delay = 0;
		setTimeout(function(){
		//sendHappy;sendSad;sendAngry;sendUncertain;sendNeutral;sendSleepy;
		intensity = 0.0;
		sendHappy();
		pitch = 0;
		yaw = 0;
		roll = 0;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
		//
		//t3 cabeza zero
		delay = delay + 700;
		setTimeout(function(){
		intensity = 0.4;
		sendHappy();
		pitch = 0;
		yaw = 100;
		roll = 100;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 50;
		rightHand = 50;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
}
function myTimer04() {
		var delay = 0;
		setTimeout(function(){
		//sendHappy;sendSad;sendAngry;sendUncertain;sendNeutral;sendSleepy;
		intensity = 0.0;
		sendAngry();
		pitch = 0;
		yaw = 0;
		roll = 40;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
		//
		//t3 cabeza zero
		delay = delay + 800;
		setTimeout(function(){
		intensity = 0.4;
		sendAngry();
		pitch = 0;
		yaw = 100;
		roll = 60;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 50;
		rightHand = 50;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
}

function myTimer05() {
		var delay = 0;
		setTimeout(function(){
		//sendHappy;sendSad;sendAngry;sendUncertain;sendNeutral;sendSleepy;
		intensity = 0.0;
		sendHappy();
		pitch = 0;
		yaw = 50;
		roll = 0;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
		//
		//t3 cabeza zero
		delay = delay + 1000;
		setTimeout(function(){
		intensity = 0.4;
		sendHappy();
		pitch = 0;
		yaw = 50;
		roll = 100;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 70;
		rightHand = 70;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		},delay); 
}

var myVar01;
var myVar02;
var myVar03;
var myVar04;
var myVar05;

function playNavidad() {
  var min = 1;
  var max = 5;
  var aleatorio =  Math.floor(Math.random() * (max - min + 1)) + min;
  switch(aleatorio){
	case 1:
		console.log ('case 1');
		flagEvent.checked = false;
		var delay = 0;
		setTimeout(function(){
		sendHappy();
		pitch = 80;
		yaw = 50;
		roll = 80;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		data = 'HABLAR:Felices fiestas!';
		sendChannel.send(data);
		},delay); 
		//
		delay = delay + 3000;
		setTimeout(function(){
		playChristmas01();
		},delay); 
		myVar01 = setInterval(myTimer01, 2000);
		//t3 cabeza zero
		delay = delay + 48000;
		setTimeout(function(){
		clearInterval(myVar01);
		pitch = 0;
		yaw = 50;
		roll = 50;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		sendSleepy();
		flagEvent.checked = true;
		},delay); 
		break;
	case 2:
		console.log ('case 2');
		flagEvent.checked = false;
		var delay = 0;
		setTimeout(function(){
		sendHappy();
		pitch = 80;
		yaw = 50;
		roll = 80;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		data = 'HABLAR:Muchas felicidades!';
		sendChannel.send(data);
		},delay); 
		//
		delay = delay + 3000;
		setTimeout(function(){
		playChristmas02();
		},delay); 
		myVar02 = setInterval(myTimer02, 2000);
		//t3 cabeza zero
		delay = delay + 47000;
		setTimeout(function(){
		clearInterval(myVar02);
		pitch = 0;
		yaw = 50;
		roll = 50;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		sendSleepy();
		flagEvent.checked = true;
		},delay); 
		break;
	case 3:
		console.log ('case 3');
		flagEvent.checked = false;
		var delay = 0;
		setTimeout(function(){
		sendHappy();
		pitch = 80;
		yaw = 50;
		roll = 80;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		data = 'HABLAR:Feliz navidad!';
		sendChannel.send(data);
		},delay); 
		//
		delay = delay + 3000;
		setTimeout(function(){
		playChristmas03();
		},delay); 
		myVar03 = setInterval(myTimer03, 1400);
		//t3 cabeza zero
		delay = delay + 34000;
		setTimeout(function(){
		clearInterval(myVar03);
		pitch = 0;
		yaw = 50;
		roll = 50;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		sendSleepy();
		flagEvent.checked = true;
		},delay); 
		break;
	case 4:
		console.log ('case 4');
		flagEvent.checked = false;
		var delay = 0;
		setTimeout(function(){
		sendAngry();
		pitch = 80;
		yaw = 50;
		roll = 80;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		data = 'HABLAR:Que la fuerza te acompañe.';
		sendChannel.send(data);
		},delay); 
		//
		delay = delay + 3000;
		setTimeout(function(){
		playChristmas04();
		},delay); 
		myVar04 = setInterval(myTimer04, 1400);
		//t3 cabeza zero
		delay = delay + 49000;
		setTimeout(function(){
		clearInterval(myVar04);
		pitch = 0;
		yaw = 50;
		roll = 50;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		sendSleepy();
		flagEvent.checked = true;
		},delay); 
		break;
	case 5:
		console.log ('case 4');
		flagEvent.checked = false;
		var delay = 0;
		setTimeout(function(){
		sendHappy();
		pitch = 80;
		yaw = 50;
		roll = 80;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 100;
		rightHand = 100;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		data = 'HABLAR:Me regalas un abrazo?!';
		sendChannel.send(data);
		},delay); 
		//
		myVar05 = setInterval(myTimer05, 2000);
		//t3 cabeza zero
		delay = delay + 8000;
		setTimeout(function(){
		clearInterval(myVar05);
		data = 'HABLAR:Gracias, me siento tan feliz!';
		sendChannel.send(data);
		pitch = 0;
		yaw = 50;
		roll = 100;
		data = 'HEADMO:'+pitch+';'+yaw+';'+roll;
		sendChannel.send(data);
		leftHand = 0;
		rightHand = 0;
		data = 'ARMSMO:'+leftHand+';'+rightHand+';0';
		sendChannel.send(data);		
		flagEvent.checked = true;
		},delay); 
		break;
  }
	
  
}
	
function sendHeadWake() {
	data = 'HEADWR:WAKE';
	sendChannel.send(data);
  }
  
function sendHeadRest() {
	data = 'HEADWR:REST';
	sendChannel.send(data);
  }
  
function baseStop() {
	data = 'ACTUAR:'+EMER+';0;0';
	sendChannel.send(data);
  }
function baseRearm() {
	data = 'ACTUAR:'+RARM+';0;0';
	sendChannel.send(data);
  }
  
  


  function applyFilter (stream) {
    console.log('ENTRO A APPLYFILTER...');
    //window.mic = context.createMediaStreamSource(stream);
    //window.mic.connect(window.filter);
    window.peer = context.createMediaStreamDestination();
    window.filter.connect(window.peer);
    return window.peer.stream;
  }


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
// INICIO
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
  
function changeInicio()
 {
	if(modeInicio.selectedIndex == 0) //
	{
		console.log('Inicio');
	}
	else if(modeInicio.selectedIndex == 1) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
		
	}
	else if(modeInicio.selectedIndex == 2) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 3) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 4) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 5) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 6) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 7) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 8) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 9) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 10) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 11) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 12) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 13) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 14) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeInicio.selectedIndex == 15) // 
	{
		data = 'HABLAR:'+modeInicio.options[modeInicio.selectedIndex].value;
		sendChannel.send(data);
	}

}
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
// DESARROLLO
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
function changeDesarrollo()
 {
	if(modeDesarrollo.selectedIndex == 0) //
	{
		console.log('Desarrollo');
	}
	else if(modeDesarrollo.selectedIndex == 1) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
		
	}
	else if(modeDesarrollo.selectedIndex == 2) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeDesarrollo.selectedIndex == 3) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 4) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 5) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 6) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 7) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 8) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 9) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 10) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 11) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 12) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 13) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 14) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 15) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 16) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 17) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 18) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 19) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeDesarrollo.selectedIndex == 20) // 
	{
		data = 'HABLAR:'+modeDesarrollo.options[modeDesarrollo.selectedIndex].value;
		sendChannel.send(data);
	} 
}  

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
// FRASES COLOQUIALES
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
  
function changeFrase()
 {
	if(modeFrase.selectedIndex == 0) //
	{
		console.log('Frase');
	}
	else if(modeFrase.selectedIndex == 1) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
		
	}
	else if(modeFrase.selectedIndex == 2) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 3) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 4) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 5) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 6) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeFrase.selectedIndex == 7) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeFrase.selectedIndex == 8) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	}
	else if(modeFrase.selectedIndex == 9) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 10) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 11) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 12) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 13) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 14) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 15) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 16) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 17) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 18) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 19) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 20) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 21) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 22) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 23) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 24) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	else if(modeFrase.selectedIndex == 25) // 
	{
		data = 'HABLAR:'+modeFrase.options[modeFrase.selectedIndex].value;
		sendChannel.send(data);
	} 
	
}
  
  
  
////////////////////////////////////////////////////////
//Callbacks for different mode selectors
////////////////////////////////////////////////////////

 //changes operating mode (Emotions, Operator, Menu)
 function changeMode()
 {
		if(modeSelector.selectedIndex == 1) //Set video operator
		{
  		happyButton.disabled = true;
  		happyButton.style.background =  "#E3E3E3";
  		happyButton.style.borderBottom = "#E3E3E3";
  		sadButton.disabled = true;
  		sadButton.style.background = "#E3E3E3";
  		sadButton.style.borderBottom = "#E3E3E3";
  		angryButton.disabled = true;
  		angryButton.style.background = "#E3E3E3";
  		angryButton.style.borderBottom = "#E3E3E3";
  		uncertainButton.disabled = true;
  		uncertainButton.style.background = "#E3E3E3";
  		uncertainButton.style.borderBottom = "#E3E3E3";
  		neutralButton.disabled = true;
  		neutralButton.style.background = "#E3E3E3";
  		neutralButton.style.borderBottom = "#E3E3E3";
  		sleepyButton.disabled = true;
  		sleepyButton.style.background = "#E3E3E3";
  		sleepyButton.style.borderBottom = "#E3E3E3";
  		intensitySlider.disabled = true;
  		intensitySlider.style.background = "#E3E3E3";
  		intensityLabel.style.color = "#E3E3E3";
      $(selectRoboticonMode).hide();
  		localVideo.style.display = 'block';
      robotIcon.style.display = 'none';
      menuMessage.style.display = 'none';
      sendChangeMode(1);
		}
		else if(modeSelector.selectedIndex == 0) // set roboticon
		{
			happyButton.disabled = false;
			happyButton.style.background = "#1E90FF";
			happyButton.style.borderBottom = "#7d7d7d";
			sadButton.disabled = false;
			sadButton.style.background = "#1E90FF";
			sadButton.style.borderBottom = "#7d7d7d";
			angryButton.disabled = false;
			angryButton.style.background = "#1E90FF";
			angryButton.style.borderBottom = "#7d7d7d";
			uncertainButton.disabled = false;
			uncertainButton.style.background = "#1E90FF";
			uncertainButton.style.borderBottom = "#7d7d7d";
			neutralButton.disabled = false;
			neutralButton.style.background = "#1E90FF";
			neutralButton.style.borderBottom =  "#7d7d7d";
			sleepyButton.disabled = false;
			sleepyButton.style.background = "#1E90FF";
			sleepyButton.style.borderBottom = "#7d7d7d";
			intensitySlider.disabled = false;
			intensitySlider.style.background = "#1E90FF";
			intensityLabel.style.color = "black";
			$(selectRoboticonMode).show().val('whole_face');
			localVideo.style.display = 'none';
			robotIcon.style.display = 'block';
			menuMessage.style.display = 'none';
		  sendChangeMode(0);
		}
		else
		{
  		happyButton.disabled = true;
  		happyButton.style.background =  "#E3E3E3";
  		happyButton.style.borderBottom = "#E3E3E3";
  		sadButton.disabled = true;
  		sadButton.style.background = "#E3E3E3";
  		sadButton.style.borderBottom = "#E3E3E3";
  		angryButton.disabled = true;
  		angryButton.style.background = "#E3E3E3";
  		angryButton.style.borderBottom = "#E3E3E3";
  		uncertainButton.disabled = true;
  		uncertainButton.style.background = "#E3E3E3";
  		uncertainButton.style.borderBottom = "#E3E3E3";
  		neutralButton.disabled = true;
  		neutralButton.style.background = "#E3E3E3";
  		neutralButton.style.borderBottom = "#E3E3E3";
  		sleepyButton.disabled = true;
  		sleepyButton.style.background = "#E3E3E3";
  		sleepyButton.style.borderBottom = "#E3E3E3";
  		intensitySlider.disabled = true;
  		intensitySlider.style.background = "#E3E3E3";
  		intensityLabel.style.color = "#E3E3E3";
  		localVideo.style.display = 'none';
  		robotIcon.style.display = 'none';
  		menuMessage.style.display = 'block';
      sendChangeMode(2);
		}
 }

 /* // Switch between chat mode or control mode
 function toggleControl()
 {
		if(!controlCheckbox.checked)
		{
  		forwardButton.disabled = true;
  		forwardButton.style.background =  "#E3E3E3";
  		forwardButton.style.borderBottom = "#E3E3E3";
  		reverseButton.disabled = true;
  		reverseButton.style.background =  "#E3E3E3";
  		reverseButton.style.borderBottom = "#E3E3E3";
  		leftButton.disabled = true;
  		leftButton.style.background =  "#E3E3E3";
  		leftButton.style.borderBottom = "#E3E3E3";
  		rightButton.disabled = true;
  		rightButton.style.background =  "#E3E3E3";
  		rightButton.style.borderBottom = "#E3E3E3";
  		/*dataChannelSend.disabled = false;
  		sendButton.disabled = false;
  		sendButton.style.background = "#1E90FF";
  		sendButton.style.borderBottom = "#7d7d7d";
		}
		else
		{
  		forwardButton.disabled = false;
  		forwardButton.style.background = "#1E90FF";
  		forwardButton.style.borderBottom = "#7d7d7d";
  		reverseButton.disabled = false;
  		reverseButton.style.background = "#1E90FF";
  		reverseButton.style.borderBottom = "#7d7d7d";
  		leftButton.disabled = false;
  		leftButton.style.background = "#1E90FF";
  		leftButton.style.borderBottom = "#7d7d7d";
  		rightButton.disabled = false;
  		rightButton.style.background = "#1E90FF";
  		rightButton.style.borderBottom = "#7d7d7d";
  		/*dataChannelSend.disabled = true;
  		sendButton.disabled = true;
  		sendButton.style.background = "#E3E3E3";
  		sendButton.style.borderBottom = "#E3E3E3";
		}
 }*/
 
 // updates intensity, which was set by the slider
 function updateIntensity()
 {
 		intensity = intensitySlider.value/100;
 }

 $(document).ready(function(){
   updateIntensity();
 });

  
/////////////////////////////////////////////////////
//Functions for different emotions
/////////////////////////////////////////////////////
 
/*function sendHappy(){
  var eR = 12 - intensity*12;
  setEmotionValues({
    eyebrowsShape: "round",
    eyebrowsRotation: eR,
    eyebrowsHeight: intensity,
    eyelidsHeight: 0,
    eyeballsDirection: 0,
    eyeballsIntensity: 0,
    mouthEmotion: "happy"
  });
}*/

function sendNeutral(){
  setEmotionValues({
    eyebrowsShape: "angular",
    eyebrowsRotation: -6.0,
    eyebrowsHeight: 0.0,
    eyelidsHeight: 0.15,
    eyeballsDirection: 0.5,
    eyeballsIntensity: 0.0,
    mouthEmotion: "neutral"
  });
}

function sendHappy(){
  var eR = -(12 - intensity*12);
  var eH = 0.6 - 0.6*intensity;
  setEmotionValues({
    eyebrowsShape: "angular",
    eyebrowsRotation: eR,//-12.0,//eR,
	eyebrowsHeight: eH,//0.0,//eH,
    //eyebrowsHeight: intensity,
    eyelidsHeight: 0.15,
    eyeballsDirection: 0,
    eyeballsIntensity: 0,
    mouthEmotion: "happy"
  });
}


function sendAsombro(){
  var eR = 12 - intensity*12;
  setEmotionValues({
    eyebrowsShape: "round",
    eyebrowsRotation: eR,
    eyebrowsHeight: intensity,
    eyelidsHeight: 0,
    eyeballsDirection: 0,
    eyeballsIntensity: 0,
    mouthEmotion: "neutral"
  });
}

function sendSad(){
  var eR = -9 - intensity*21;
  var eH = 0.4 + 0.2*intensity;
  setEmotionValues({
    eyebrowsShape: "angular",
    eyebrowsRotation: eR,
    eyebrowsHeight: eH,
    eyelidsHeight: 0.55,
    eyeballsDirection: 0,
    eyeballsIntensity: 0.35,
    mouthEmotion: "sad"
  });
}

function sendAngry(){
  var eR = 5 + intensity*35;
  var eH = 0.6 - 0.6*intensity;
  setEmotionValues({
    eyebrowsShape: "angular",
    eyebrowsRotation: eR,
    eyebrowsHeight: eH,
    eyelidsHeight: 0.15,
    eyeballsDirection: 0,
    eyeballsIntensity: 0,
    mouthEmotion: "angry"
  });
}

function sendUncertain(){
  var eR = -2 - intensity*28;
  
  setEmotionValues({
    eyebrowsShape: "angular",
    eyebrowsRotation: eR,
    eyebrowsHeight: 0.5,
    eyelidsHeight: 0,
    eyeballsDirection: 119,
    eyeballsIntensity: 0,
    mouthEmotion: "uncertain"
  });
}


function sendSleepy(){
	var eH = -intensity*0.5;
	var elH = 0.25 + 0.75*intensity;
	
	setEmotionValues({
    eyebrowsShape: "round",
    eyebrowsRotation: -9,
    eyebrowsHeight: eH,
    eyelidsHeight: elH,
    eyeballsDirection: 0,
    eyeballsIntensity: 0.3,
    mouthEmotion: "neutral"
  });
}

function setEmotionValues(emotionValues) {
  
  var defaultEmotionValues = {
    eyebrowsShape: "round",
    eyebrowsRotation: 90,
    eyebrowsHeight: 0.5,
    eyelidsHeight: 0.5,
    eyeballsDirection: 60,
    eyeballsIntensity: 0.5,
    mouthEmotion: "neutral"
  };

  var mergedValues = jQuery.extend(true, {}, defaultEmotionValues, emotionValues)
  var jsonChanges = { eyebrows: {
                        shapes: {
                          both_sides: mergedValues.eyebrowsShape
                        },
                        transform: {
                          both_sides: {
                            rotation: mergedValues.eyebrowsRotation,
                            height: mergedValues.eyebrowsHeight
                          }
                        },
                      },
                      eyelids: {
                        heights: {
                          both_sides: mergedValues.eyelidsHeight
                        },
                      },
                      eyeballs: {
                        positions: {
                          both_sides: {
                            direction: mergedValues.eyeballsDirection,
                            intensity: mergedValues.eyeballsIntensity
                          },
                        },
                      },
                      mouth: {
                        emotion: mergedValues.mouthEmotion
                      }
                    };

  changeEmotion(jsonChanges);
}

// Send JSON string with emotion
function changeEmotion(jsonChanges) {
  var s = JSON.stringify(jsonChanges);
  RobotIcon.parseAndApplyJson(s)
  var data = 'ROBICO:'+s;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}

function changeRoboticonDisplayMode() {
  var mode = selectRoboticonMode.value;
  RobotIcon.changeDisplayMode(mode);
  sendChannel.send('ICOMOD:' + mode);
}

function sendChangeMode(mode) {
  var data = 'MODECH:'+mode;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}

function sendSocialModeOn() {
  var data = 'ACTUAR:'+1+';'+SMON;  
  msgTxt.value = 'sendSocialModeOn: ' + data;
  sendChannel.send(data);
}
function sendSocialModeOff() {
  var data = 'ACTUAR:'+1+';'+SMOF;  
  msgTxt.value = 'sendSocialModeOff: ' + data;
  sendChannel.send(data);
}

function sendPidConservative() {
  var data = 'ACTUAR:'+1+';'+PIDC;  
  msgTxt.value = 'sendPidConservative: ' + data;
  sendChannel.send(data);
}

function sendPidAggresive() {
  var data = 'ACTUAR:'+1+';'+PIDA;  
  msgTxt.value = 'sendPidAggresive: ' + data;
  sendChannel.send(data);
}

function changeFlagEvent() {
  console.log('FlagEvent:' + flagEvent.checked);
}

  
  

var audioInputSelect = document.querySelector('select#audioSource');
var audioOutputSelect = document.querySelector('select#audioOutput');
var videoSelect = document.querySelector('select#videoSource');
var selectors = [audioInputSelect, audioOutputSelect, videoSelect];

navigator.mediaDevices.enumerateDevices().then(gotDevices).catch(handleError);

function gotDevices(deviceInfos) {
	console.log ('Entro a gotDevices');
  // Handles being called several times to update labels. Preserve values.
  for (var i = 0; i !== deviceInfos.length; ++i) {
    var deviceInfo = deviceInfos[i];
    //var option = document.createElement('option');
    //option.value = deviceInfo.deviceId;
    if (deviceInfo.kind === 'audioinput') {
      //option.text = deviceInfo.label ||
      //    'microphone ' + (audioInputSelect.length + 1);
		  console.log('audioinput', deviceInfo.label ||
          'microphone ' + (audioInputSelect.length + 1));
      //audioInputSelect.appendChild(option);
    } else if (deviceInfo.kind === 'audiooutput') {
      //option.text = deviceInfo.label || 'speaker ' +
        //  (audioOutputSelect.length + 1);
		  console.log('audiooutput',deviceInfo.label || 'speaker ' +
          (audioOutputSelect.length + 1));
      //audioOutputSelect.appendChild(option);
    } else if (deviceInfo.kind === 'videoinput') {
      //option.text = deviceInfo.label || 'camera ' + (videoSelect.length + 1);
      //videoSelect.appendChild(option);
	  console.log('videoinput', deviceInfo.label || 'camera ' + (videoSelect.length + 1));
    } else {
      console.log('Some other kind of source/device: ', deviceInfo);
    }
  }
  // selectors.forEach(function(select, selectorIndex) {
    // if (Array.prototype.slice.call(select.childNodes).some(function(n) {
      // return n.value === values[selectorIndex];
    // })) {
      // select.value = values[selectorIndex];
    // }
  // });
}


function handleSuccess(stream) {
  var videoTracks = stream.getVideoTracks();
  //console.log('Got stream with constraints:', constraints);
  console.log('Using video device: ' + videoTracks[0].label);
  stream.oninactive = function() {
    console.log('Stream inactive');
  };
  window.stream = stream; // make variable available to browser console
//  localVideo.srcObject = stream;
  localVideo.src = window.URL.createObjectURL(stream);
  localStreamFrontal = stream;
  
  
}

function handleError(error) {
  if (error.name === 'ConstraintNotSatisfiedError') {
    // errorMsg('The resolution ' + constraints.video.width.exact + 'x' +
        // constraints.video.width.exact + ' px is not supported by your device.');
		console.log('Resolution error');
  } else if (error.name === 'PermissionDeniedError') {
    console.log('Permissions have not been granted to use your camera and ' +
      'microphone, you need to allow the page access to your devices in ' +
      'order for the demo to work.');
  }
  errorMsg('getUserMedia error: ' + error.name, error);
}


function handleSuccessDummy(stream) {
  console.log('Got stream with constraints:');
}

function handleErrorDummy(error) {
  console.log('getUserMedia error ');
}


function errorMsg(msg, error) {
  //errorElement.innerHTML += '<p>' + msg + '</p>';
  //if (typeof error !== 'undefined') {
    console.error(error);
  //}
}
  