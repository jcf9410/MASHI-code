twitterButton.onclick = sendTweet;
var dataTweetImg;

function sendTweet() {
  console.log ('sendTweet..');
  console.log (dataTweetImg);
  //var data = btoa(dataTweetImg);
  //console.log (data)
	data = 'HABLAR:OK, thanks!';
	sendChannel.send(data);
  
  socket.emit('TWEET', dataTweetImg);
}
