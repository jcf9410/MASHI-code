var legoBuffer = null;
var christmas01Buffer = null;
var christmas02Buffer = null;
var christmas03Buffer = null;
var christmas04Buffer = null;

var lego = document.querySelector('button#lego');
var christmas01 = document.querySelector('button#christmas01');
var christmas02 = document.querySelector('button#christmas02');
var christmas03 = document.querySelector('button#christmas03');
var christmas04 = document.querySelector('button#christmas04');

lego.onclick = playLego;
christmas01.onclick = playChristmas01;
christmas02.onclick = playChristmas02;
christmas03.onclick = playChristmas03;
christmas04.onclick = playChristmas04;

loadLego ('/audio/lego.wav');
loadChristmas01 ('/audio/christmas01.mp3');
loadChristmas02 ('/audio/christmas02.mp3');
loadChristmas03 ('/audio/christmas03.mp3');
loadChristmas04 ('/audio/christmas04.mp3');

function playLego() {
	//  webAudio.addEffect();
	//playSound (dogBarkingBuffer);
	console.log ('ENTRO A PLAYSOUNDSIRENA..');
    var effect = context.createBufferSource();
    effect.buffer = legoBuffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      console.log ('ENTRO A WINDOW.PEER..');
      effect.connect(window.peer);
      effect.start(0);
    }
}
function loadLego(url) {
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      legoBuffer = buffer;
    }, onError);
  }
  request.send();
}

function playChristmas01() {
    var effect = context.createBufferSource();
    effect.buffer = christmas01Buffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      effect.connect(window.peer);
      effect.start(0);
    }
}
function loadChristmas01(url) {
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      christmas01Buffer = buffer;
    }, onError);
  }
  request.send();
}

function playChristmas02() {
    var effect = context.createBufferSource();
    effect.buffer = christmas02Buffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      effect.connect(window.peer);
      effect.start(0);
    }
}
function loadChristmas02(url) {
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      christmas02Buffer = buffer;
    }, onError);
  }
  request.send();
}

function playChristmas03() {
    var effect = context.createBufferSource();
    effect.buffer = christmas03Buffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      effect.connect(window.peer);
      effect.start(0);
    }
}
function loadChristmas03(url) {
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      christmas03Buffer = buffer;
    }, onError);
  }
  request.send();
}


function playChristmas04() {
    var effect = context.createBufferSource();
    effect.buffer = christmas04Buffer;
    //window.peer = context.createMediaStreamDestination();
    if (window.peer) {
      effect.connect(window.peer);
      effect.start(0);
    }
}
function loadChristmas04(url) {
  var request = new XMLHttpRequest();
  request.open('GET', url, true);
  request.responseType = 'arraybuffer';
  // Decode asynchronously
  request.onload = function() {
    context.decodeAudioData(request.response, function(buffer) {
      christmas04Buffer = buffer;
    }, onError);
  }
  request.send();
}

