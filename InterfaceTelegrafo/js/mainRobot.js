/*
20160701: MediaStreamTrack.getSources() deprecated. To solve
*/
'use strict';

var DEBUG = 0XFF; // Debug Print.  Set to FF to debug

var data;
var habilitarOmni = false; // bandera para habilitar Camara Omni (true) o deshabilitar (false)


//Variables para configurar datachannel
var sendChannel, receiveChannel, pcConstraint, dataConstraint;
var dataChannelSend = document.querySelector('textarea#dataChannelSend');
var dataChannelReceive = document.querySelector('textarea#dataChannelReceive');
var sctpSelect = document.querySelector('input#useSctp');
var rtpSelect = document.querySelector('input#useRtp');
var startButton = document.querySelector('button#startButton');
var sendButton = document.querySelector('button#sendButton');
var closeButton = document.querySelector('button#closeButton');
////////////////////////////////////////////
// Variables para botones de control del robot
////////////////////////////////////////////////
/*var forwardButton = document.querySelector('button#forward');
var leftButton = document.querySelector('button#left');
var stopButton = document.querySelector('button#stop');
var rightButton = document.querySelector('button#right');
var reverseButton = document.querySelector('button#reverse');
*/
////////////////////////////////////////////////////

var localVideo = document.querySelector('#localVideo'); //Frontal camera
var localVideo2 = document.querySelector('#localVideo2');
var remoteVideo = document.querySelector('#remoteVideo');

////////////////////////////////////////////////////
var existeCamaraFrontal;
var existeCamaraOmni;

//startButton.onclick = createConnection;
sendButton.onclick = sendData;

/*forwardButton.onclick = sendDataForward;
leftButton.onclick = sendDataLeft;
stopButton.onclick = sendDataStop;
rightButton.onclick = sendDataRight;
reverseButton.onclick = sendDataReverse;
*/
dataChannelSend.onkeypress = handleSendKeyPress;
/*closeButton.onclick = closeDataChannels;
rtpSelect.onclick = enableStartButton;
sctpSelect.onclick = enableStartButton;
*/

var user='robot';
var isChannelReady;
var isInitiator = true;
var isStarted = false;
var localStream, localStreamFrontal, localStreamOmni;
var pc;
var remoteStream;
var turnReady;

var constraints;
//var pc_config = {'iceServers': [{url:'stun:stun.l.google.com:19302' }] };

var pc_config = {
  'iceServers': [
    {url: 'stun:stun.l.google.com:19302'},
	{url:'stun:stun01.sipphone.com'},
{url:'stun:stun.ekiga.net'},
{url:'stun:stun.fwdnet.net'},
{url:'stun:stun.ideasip.com'},
{url:'stun:stun.iptel.org'},
{url:'stun:stun.rixtelecom.se'},
{url:'stun:stun.schlund.de'},
{url:'stun:stun.l.google.com:19302'},
{url:'stun:stun1.l.google.com:19302'},
{url:'stun:stun2.l.google.com:19302'},
{url:'stun:stun3.l.google.com:19302'},
{url:'stun:stun4.l.google.com:19302'},
{url:'stun:stunserver.org'},
{url:'stun:stun.softjoys.com'},
{url:'stun:stun.voiparound.com'},
{url:'stun:stun.voipbuster.com'},
{url:'stun:stun.voipstunt.com'},
{url:'stun:stun.voxgratia.org'},
{url:'stun:stun.xten.com'},
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
}

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
});

//////////////////////////////////////////////////////////////////////////////

var socket = io.connect();
var room = 'robotRoom';
if (room !== '') {
  console.log('create or join', room);
  socket.emit('create or join', room);
}



socket.on('created', function (room){
  // Aqui entra el iniciador de la conversacion ei robot
  isInitiator = true;
  user = 'robot';
  console.log('Created: ' + user + ' isInitiator: ' + isInitiator);
  ////////////////////////////////////////////////
  // getUserMedia para camara frontal
  ////////////////////////////////////////////////
  
  //buscar id de camara frontal 'Logitech QuickCam Pro 5000 (046d:08c5)'
/*
      media_sources.forEach(function (media_source) {
        if (media_source.kind === 'video') {
            console.log("media_source..." + media_source.label);
            //videoSources.push(media_source);
        }
    });
*/
    //Dummy constraints
    constraints = { audio: true , video: { mandatory: { maxWidth: 320, maxHeight: 180}}};
	//constraints = { video: false, audio: true };
	navigator.mediaDevices.getUserMedia(constraints).
    then(handleSuccessDummy).catch(handleErrorDummy);
    //getUserMedia(constraints, handleUserMediaDummy, handleUserMediaDummyError);
    videoSources.forEach(function (video_source) {
       console.log("video_sources for each..." + video_source.label);
	   	//if (true){
		if (video_source.label === 'Webcam C170 (046d:082b)') {	
		//if (video_source.label === 'Logitech HD Pro Webcam C920 (046d:082d)') {	
		//if (video_source.label === 'TOSHIBA Web Camera - HD (04f2:b3b1)') {	
		//if (video_source.label === 'USB2.0 HD UVC WebCam (04f2:b354)') {	
		//if (video_source.label === 'Logitech QuickCam Pro 5000 (046d:08c5)') {	
		//if (video_source.label === 'Microsoft LifeCam VX-3000 (045e:00f5)') {
            console.log("Frontal camera: " + video_source.label);
            //GetUserMedia de camara frontal
            //basic constraints
            //constraints = { video: true, audio: true };
            //QVGA resultante : 176x144
			//constraints = { audio: true, video: { mandatory: { maxWidth: 160, maxHeight: 120, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "20"}} };
            //constraints = { audio: true, video: { mandatory: { maxWidth: 320, maxHeight: 180, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "30"}} };
            //VGA video Stream resultante: 352x288
            //constraints = { audio: true, video: { mandatory: { maxWidth: 640, maxHeight: 360, googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id, "maxFrameRate": "30"} } };
            //HD video 1280x720

			//Webcam C170 
			
			//var constraints = window.constraints{ audio: true, video: { mandatory: { maxWidth: 640, maxHeight: 480, googCpuOveruseDetection: true, sourceId : video_source.id, "maxFrameRate": "10"} } };			
			
			
			//constraints = { audio: true, video: { mandatory: { maxWidth: 351, maxHeight: 263, googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id, "maxFrameRate": "10"} } };			
			//Webcam C170 600 x 450
			//constraints = { audio: true, video: { mandatory: { maxWidth: 600, maxHeight: 450, googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id, "maxFrameRate": "10"} } };			
			////Webcam C170 640 x 480
			var constraints = window.constraints =  { audio: true, video: { mandatory: { maxWidth: 640, maxHeight: 480,  sourceId : video_source.id, "maxFrameRate": "10"} } };			
			//constraints = { video: false, audio: true };
			
			//constraints = { audio: true, video: { mandatory: { maxWidth: 351, maxHeight: 263, googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id, "maxFrameRate": "30"} } };			

			//constraints = { audio: true, video: { mandatory: { minWidth: 1280, minHeight: 720,googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id} } };			
			//constraints = { audio: true, video: { mandatory: { minWidth: 640, minHeight: 480,googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id} } };			

			//onstraints = { audio: true, video: { mandatory: { minWidth: 1280, minHeight: 720, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id} } };
            //console.log(user + ' getUserMedia, constraints', constraints);
            //getUserMedia(constraints, handleUserMediaFrontal, handleUserMediaFrontalError);
			navigator.mediaDevices.getUserMedia(constraints).
			then(handleSuccess).catch(handleError);
        }
        else if (video_source.label === 'Logitech HD Pro Webcam C920 (046d:082d)') {
		//else if (video_source.label === 'Trust WB-1300N Webcam Live (145f:013a)') {
		//else if (video_source.label === 'VGA USB Camera (093a:2621)') {
         console.log("Omnidirectional con id..." + video_source.id);
			       habilitarOmni=true;
            //GetUserMedia de camara frontal
            //basic constraints
            //constraints = { video: true, audio: true };
            //QVGA resultante : 176x144
			//constraints = { audio: false, video: { mandatory: { maxWidth: 320, maxHeight: 180, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "30"}}};
            //Minimun resolution for Trust WB-1300N
			////constraints = { audio: false, video: { mandatory: { maxWidth: 160, maxHeight: 120, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "20"}}};
            //VGA video Stream resultante: 352x288
			//constraints = { audio: false, video: { mandatory: { maxWidth: 640, maxHeight: 360, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "20"}}};
  			//constraints = { audio: true, video: { mandatory: { maxWidth: 640, maxHeight: 360, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id } } };
            //HD video 1280x720
			//constraints = { audio: false, video: { mandatory: { maxWidth: 468, maxHeight: 351, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "20"}}};
            //constraints = { audio: true, video: { mandatory: { minWidth: 1280, minHeight: 720, googCpuOveruseDetection: true, googLeakyBucket: true } } };
            //console.log(user + ' getUserMedia, constraints', constraints);
			
			// Webcam C920. Full HD panoramic view constraint 1280x720
			//constraints = { audio: true, video: { mandatory: { minWidth: 1280, minHeight: 720,googCpuOveruseDetection: true, googLeakyBucket: true  , sourceId : video_source.id, "maxFrameRate": "10"} } };			
			// Webcam C920. Panoramic view constraint 432 x 240
			//constraints = { audio: false, video: { mandatory: { maxWidth: 468, maxHeight: 351, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "20"}}};
			// Webcam C920. Panoramic view constraint 600 x 333
			//constraints = { audio: false, video: { mandatory: { maxWidth: 600, maxHeight: 333, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};
			// Webcam C920. Panoramic view constraint 640 x 360
			constraints = { audio: false, video: { mandatory: { maxWidth: 640, maxHeight: 360, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};
			//constraints = { audio: false, video: { mandatory: { maxWidth: 700, maxHeight: 389, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};
			// Webcam C920. Panoramic view constraint 800 x 444 Aqui cambia la configuracion a 640 x 444
			//constraints = { audio: false, video: { mandatory: { maxWidth: 800, maxHeight: 444, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};
			// Webcam C920. Panoramic view constraint 1000 x 555 Aqui cambia la configuracion a 640 x 480
			//constraints = { audio: false, video: { mandatory: { maxWidth: 1000, maxHeight: 555, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};
			// Webcam C920. Panoramic view constraint 1500 x 833 Aqui cambia la configuracion a 640 x 480
			//constraints = { audio: false, video: { mandatory: { maxWidth: 1500, maxHeight: 833, googCpuOveruseDetection: true, googLeakyBucket: true , sourceId : video_source.id, "maxFrameRate": "10"}}};

			
            //getUserMedia(constraints, handleUserMediaOmni, handleUserMediaOmniError);
			navigator.mediaDevices.getUserMedia(constraints).
    then(handleSuccess).catch(handleError);

        }
    });

    if (isInitiator){
      maybeStart();
    }
  
});


socket.on('join', function (room){
  // aqui entra el robot
  console.log('JOIN user ' + user);
  isChannelReady = true;
});

socket.on('joined', function (room){
  //aqui entra dennys
    console.log('JOINED user ' + user);
    isChannelReady = true;
    //console.log('isChannelReady ' + isChannelReady);

    //QVGA resultante : 176x144
    var constraints = window.constraints = { audio: true, video: { mandatory: { maxWidth: 320, maxHeight: 180} } };
    //constraints = { audio: true, video: { mandatory: { maxWidth: 160, maxHeight: 120, googCpuOveruseDetection: true, googLeakyBucket: true } } };
    //constraints = { audio: true, video: false};

    //getUserMedia(constraints, handleUserMedia, handleUserMediaError);
	navigator.mediaDevices.getUserMedia(constraints).
    then(handleSuccess).catch(handleError);

//    console.log(user + ' getUserMedia con constraints', constraints);

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
  localStream = stream;
  sendMessage('got user media');
  console.log('handleUserMedia: '+ user+ ' isInitiator: ' + isInitiator);
  if (isInitiator) {
    maybeStart();
  }
}

function handleUserMediaError(error){
  console.log('Error de getUserMedia: ', error);
}


function handleUserMediaFrontal(stream) {
  console.log(user + ' handleUserMediaFrontal');
  localVideo.src = window.URL.createObjectURL(stream);
  localStreamFrontal = stream;
  sendMessage('got user media de camara frontal');
/*  if (isInitiator) {
    //maybeStart();
  }
*/}

function handleUserMediaFrontalError(error){
  console.log('Error de getUserMediaFrontal: ', error);
}


function handleUserMediaOmni(stream) {
  console.log(user + ' handleUserMediaOmni');
  localVideo2.src = window.URL.createObjectURL(stream);
  localStreamOmni = stream;
  sendMessage('got user media de camara omni');
  /*if (isInitiator) {
    //maybeStart();
  }*/
}

function handleUserMediaOmniError(error){
  console.log('Error de getUserMedia omnidireccional: ', error);
}



function handleUserMediaDummy(stream) {
  console.log(user + ' handleUserMediaDummy');
  //localVideo.src = window.URL.createObjectURL(stream);
  //localStream = stream;
  //sendMessage('got user media de camara frontal');

}

function handleUserMediaDummyError(error){
  console.log('Error de getUserMediaDummy: ', error);
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
  console.log('maybeStart User ' + user);
  console.log('!isStarted=' + !isStarted + ' localStream: ' + localStream + ' isChannelReady ' + isChannelReady);
  //if (!isStarted && typeof localStreamFrontal != 'undefined' && typeof localStreamOmni != 'undefined' && isChannelReady) {
  if (!isStarted && typeof localStreamFrontal != 'undefined' && isChannelReady) 
  {
    console.log('maybeStart ' + user + ' entra al if');
    createPeerConnection();
    createDataChannel();

    pc.ondatachannel = receiveChannelCallback;

    sendChannel.onopen = onSendChannelStateChange;
    sendChannel.onclose = onSendChannelStateChange;

    //Primero se añade el stream de la camara frontal
    pc.addStream(localStreamFrontal);
    //Segundo se añade el stream de la camara omnidireccional
  	if (habilitarOmni == true){
		console.log ('habilitarOmni == true');
  		pc.addStream(localStreamOmni);
    }
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

function handleRemoteStreamAdded(event) {
  console.log(user + ' Remote stream added.');
  remoteVideo.src = window.URL.createObjectURL(event.stream);
  remoteStream = event.stream;
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
  console.log(user + ' Remote stream added.');
  remoteVideo.src = window.URL.createObjectURL(event.stream);
  remoteStream = event.stream;
}

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
  if (dataChannelReceive.value.substr(0, 7) === 'ACTUAR:'){
    console.log(user + ' ACTUAR ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    socket.emit('ACTUAR', dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'HEADMO:'){
  	if (DEBUG){
		console.log(user + ' HEADMO ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
	}	
    socket.emit('HEADMO', dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'ARMSMO:'){
  	if (DEBUG){
		console.log(user + ' ARMSMO ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
	}	
    socket.emit('ARMSMO', dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'HEADWR:'){
  	if (DEBUG){
		console.log(user + ' HEADWR ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
	}	
    socket.emit('HEADWR', dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'HABLAR:'){
    console.log(user + ' HABLAR ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    speak(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'ROBICO:'){
    console.log(user + ' ROBICO ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    change_emotion(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'ICOMOD:'){
    console.log(user + ' ICOMOD ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    change_roboticon_mode(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'MODECH:'){
    console.log(user + ' MODECH ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    change_mode(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  else if (dataChannelReceive.value.substr(0, 7) === 'SROVOL:'){
    console.log(user + ' SROVOL = ' + dataChannelReceive.value.substr(7,dataChannelReceive.value.length))
    change_robotVol(dataChannelReceive.value.substr(7,dataChannelReceive.value.length));
  }
  
}


function onSendChannelStateChange() {
  var readyState = sendChannel.readyState;
  trace('onSendChannelStateChange Send channel state is: ' + readyState);
/*  if (readyState == 'open') {
    dataChannelSend.disabled = false;
    dataChannelSend.focus();
    sendButton.disabled = false;
    closeButton.disabled = false;
  } else {
    dataChannelSend.disabled = true;
    sendButton.disabled = true;
    closeButton.disabled = true;
  }
*/}
  

function onReceiveChannelStateChange() {
  var readyState = receiveChannel.readyState;
  trace('Receive channel state readyState is: ' + readyState);
}


//funciones para enviar datos al robot mediante el datachannel

function handleSendKeyPress(event) {
  console.log ('entro a handleSendKeyPress...' + event.keyCode + ' '+ event.which);
  var key=event.keyCode || event.which;
  if (key==13){
    console.log('entro a key13..');
    sendData();
   
  }
}



function sendData() {
  var data = 'HABLAR:'+dataChannelSend.value;
  console.log ('entro a sendData');
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
  dataChannelSend.value = null;
}


function sendDataForward() {
  var data = 'ACTUAR:forward';
  //dataChannelSend.value = data;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}

function sendDataLeft() {
  var data = 'ACTUAR:left';
  //dataChannelSend.value = data;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}

function sendDataStop() {
  var data = 'ACTUAR:stop';
  //dataChannelSend.value = data;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}
function sendDataRight() {
  var data = 'ACTUAR:right';
  //dataChannelSend.value = data;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}

function sendDataReverse() {
  var data = 'ACTUAR:reverse';
  //dataChannelSend.value = data;
  sendChannel.send(data);
  trace(user + ' envia dato: ' + data);
}


function change_emotion(jsonStr) {
  console.log(jsonStr);
  RobotIcon.parseAndApplyJson(jsonStr);
 }

 function change_roboticon_mode(mode) {
    RobotIcon.changeDisplayMode(mode);
 }

function change_mode(mode) {
  if (mode == 1) {
    fullscreenControl.setVideo();
  } else if (mode == 0) {
    fullscreenControl.setRoboticon();
  } else {
    fullscreenControl.setNone();
  }
 }

 function change_robotVol(volumen) {
	remoteVideo.volume = volumen;
 }
 
window.addEventListener('load', init, false);

function init() {
  try {
	console.log ('mainRobot load ini');
    // Fix up for prefixing
	remoteVideo.volume = 0;	 
	}
  catch(e) {
    alert('load init error');
  }
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



