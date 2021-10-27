/*
---- CODE SOURCE ----
[HTML5 Video Capturing]        -> https://www.html5rocks.com/en/tutorials/getusermedia/intro/
[HTML5 Video Capturing+Saving] -> https://gist.github.com/prof3ssorSt3v3/48621be79794a8a3adeed7971786d4d8
---- CODE SOURCE ----
*/

// Initialize variables related to the video recording
var recordedVids = 1;
var vids = []
var goalList = []

var video = document.querySelector("#videoElement");

const videoSelect = document.getElementById('cameraSelect');

navigator.mediaDevices.enumerateDevices().then(gotDevices).then(getStream).catch(function(err){console.log(err)});

videoSelect.onchange = getStream;

// gotDevices(deviceInfos)
// Checks the available cameras and updates the videoSelect dropdown menu
function gotDevices(deviceInfos) {
    const option = document.createElement('option');
    option.text = 'Off';
    option.value = -1;
    videoSelect.appendChild(option);
    for (let i = 0; i !== deviceInfos.length; ++i) {
        const deviceInfo = deviceInfos[i];
        const option = document.createElement('option');
        option.value = deviceInfo.deviceId;
        if (deviceInfo.kind === 'videoinput') {
            console.log(videoSelect.length);
            option.text = 'Camera ' + (videoSelect.length);
            videoSelect.appendChild(option);
        }
    }
}

// getStream()
// When a camera is selected from the dropdown menu starts the media stream to video component
function getStream() {
    console.log(videoSelect.value)
    if (video.srcObject) {
            video.srcObject.getTracks().forEach(function(track) {
            track.stop();
        });
    }

    if(videoSelect.value != -1){
        const constraints = {
            audio: false, 
            video: {
                deviceId: {exact: videoSelect.value}
            }
        };
        navigator.mediaDevices.getUserMedia(constraints).then(gotStream).catch(function(err){console.log(err)});
    }
    
}

// gotStream(stream)
// Records the video stream
// Recording starts when Start button in demo section is pressed
// Recording stops when the function recieves an 'demoTimePaused' event 
function gotStream(stream) {
    video.srcObject = stream;

    let start = document.getElementById('demoTimerStart');
    // mediaRecorder settings 
    var mediaRecorder = new MediaRecorder(stream);
    let chunks = [];
        
    start.addEventListener('click', (ev)=>{
        mediaRecorder.start();
        console.log(mediaRecorder.state);
    })
    window.addEventListener('demoTimePaused', (ev)=>{
        mediaRecorder.stop();
        console.log(mediaRecorder.state);
    });
    mediaRecorder.ondataavailable = function(ev) {
        chunks.push(ev.data);
    }
    mediaRecorder.onstop = (ev)=>{
        // blob settings
        let blob = new Blob(chunks, { 'type' : 'video/mp4;' });
        chunks = [];
        let videoURL = window.URL.createObjectURL(blob);
        vids.push(videoURL);
        updateRecordedVideos(videoURL);
    }
}

// updateRecordedVideos(url)
// Places the recoded video in HTML and creates the download button
function updateRecordedVideos(url){
    var goalText = ""
    if(videoGoal){
        goalText = " | GOAL";
        videoGoal = false;
        goalList.push(1)
    }else{
        goalList.push(0);
    }

    var recordedVideoAccordion = document.getElementById("accordion")
    var cardContent = document.createElement('div');
    cardContent.className = "card";
    cardContent.innerHTML = 
    '<div class="card-header" id="videoHeading"'+recordedVids+'>'+
    '<h5 class="mb-0">'+
        '<button class="btn btn-link" data-toggle="collapse" data-target="#video'+recordedVids+'" aria-expanded="true" aria-controls="video"'+recordedVids+'>'+
        'Group '+groupNo+' | Video '+recordedVids+goalText+
        '</button>'+
        '<button type="button" class="btn btn-danger btn-sm" onclick="downloadVideo('+(recordedVids-1)+')">Download</button>'+
    '</h5>'+
    '</div>'+
    '<div id="video'+recordedVids+'" class="collapse" aria-labelledby="videoHeading'+recordedVids+'" data-parent="#accordion">'+
    '<div class="card-body">'+
        '<center><video autoplay="true" src='+url+' controls></video></center>'+
    '</div>'+
    '</div>';


    recordedVideoAccordion.append(cardContent)
    if(recordedVids == 1){
        document.getElementById('recordedVideos').style = "display: block;";
    }
    recordedVids += 1;
}

// downloadVideo(vid)
// Creates the link to download the recorded streams
function downloadVideo(vid){
    var a = document.createElement("a");
    a.href = vids[vid];
    if(goalList[vid]){
        a.setAttribute("download", "Field"+fieldNo+"_Group"+groupNo+"_Video"+(vid+1)+"_GOAL");
        videoGoal = false;
    }else{
        a.setAttribute("download", "Field"+fieldNo+"_Group"+groupNo+"_Video"+(vid+1));
    }
    
    a.click();
}
