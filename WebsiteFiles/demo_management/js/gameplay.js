// Load game sounds from files 
var whistle = new Audio('sounds/whistle.wav');
var bell = new Audio('sounds/bell.wav');
var horn = new Audio('sounds/airhorn.wav');

// Custom event to be trigger when demo time is paused
var pauseEvent = new CustomEvent('demoTimePaused');

// onStartSetupTimer()
// Setup Timer
// Loops in each 100 ms (0.1 s)
function onStartSetupTimer(){
    // Disable Start button
    document.getElementById("setupTimerStart").disabled = true;
    elapsed_seconds = 0;

    var x = setInterval(function() {
        elapsed_seconds += 0.1;
        difference = setupDuration - elapsed_seconds;

        var minutes = Math.floor(difference/60);
        var seconds = Math.floor(difference%60);

        document.getElementById("setupTimer").innerHTML = minutes + " m " + seconds + " s";

        // Reset Field stops the timer
        if(setupTimePaused){
            console.log("Setup Time Reset!")
            clearInterval(x);
        }

        // Play sounds and update HTML components
        if (difference < 0) {
            whistle.play();
            clearInterval(x);
            document.getElementById("setupTimer").innerHTML = "Expired";
            document.getElementById("demoTimerStart").disabled = false;
            document.getElementById("timeoutTimerStart").disabled = false;
        }else if (difference > (setupWarning-0.1) && difference < (setupWarning+0.1)) {
            bell.play();
        }
    }, 100);
    console.log("Started Setup Timer!")
}

// onStartDemoTimer()
// Demo Timer
// Loops in each 100 ms (0.1 s)
function onStartDemoTimer(){
    demoTimePaused = false;

    // Update button status 
    document.getElementById("demoTimerStart").disabled = true;
    document.getElementById("demoTimerPause").disabled = false;

    document.getElementById("openSampleButton").disabled = false;
    document.getElementById("rockSampleButton").disabled = false;
    document.getElementById("penaltyButton").disabled = false;

    var duration = demoTimeRemaining;
    elapsed_seconds = 0;

    var x = setInterval(function() {
        elapsed_seconds += 0.1;
        difference = duration - elapsed_seconds;

        var minutes = Math.floor(difference/60);
        var seconds = Math.floor(difference%60);

        document.getElementById("demoTimer").innerHTML = minutes + " m " + seconds + " s";

        // Demo can be caused by Pause, Goal or Reset Field buttons
        if(demoTimePaused){
            console.log("Demo Paused");
            clearInterval(x);
            // If paused by Goal button save goal information and update score

            // if(goalScored){
            //     var goalTime = demoDuration - difference;
            //     var goalRate = demoTimeLastPausedAt - difference;
            //     updateGoalInfo(goalTime, goalRate);
            //     goalScored = false;
            //     videoGoal = true;
            // }
            demoTimeRemaining = difference;
            demoTimeLastPausedAt = difference;

            // Notify video recorder to stop recording due to a pause in demo time
            window.dispatchEvent(pauseEvent);
        }

        // Play sounds and update HTML components
        if (difference < 0) {
            whistle.play();
            clearInterval(x);
            document.getElementById("demoTimer").innerHTML = "Expired";
            document.getElementById("demoTimerStart").disabled = true;
            document.getElementById("demoTimerPause").disabled = true;
            document.getElementById("timeoutTimerStart").disabled = true;
            document.getElementById("timeoutTimerPause").disabled = true;
            document.getElementById("openSampleButton").disabled = true;
            document.getElementById("rockSampleButton").disabled = true;
            document.getElementById("penaltyButton").disabled = true;
            window.dispatchEvent(pauseEvent);
        }else if (difference > (demoWarning-0.1) && difference < (demoWarning+0.1)) {
            console.log(difference)
            bell.play();
        }
    }, 100);
    console.log("Started Demo Timer!")
}

// onPauseDemoTimer()
// Pause button activates this function
// Updates demoTimePaused variable and updates HTML componenets 
function onPauseDemoTimer(){
    demoTimePaused = true;
    document.getElementById("demoTimerPause").disabled = true;
    document.getElementById("demoTimerStart").disabled = false;
    document.getElementById("openSampleButton").disabled = true;
    document.getElementById("rockSampleButton").disabled = true;
    document.getElementById("penaltyButton").disabled = true;
}

// onStartTimeoutTimer()
// Timeout Timer
// Loops in each 100 ms (0.1s)
function onStartTimeoutTimer(){
    timeoutTimePaused = false;
    onPauseDemoTimer();

    // Update button status
    document.getElementById("timeoutTimerStart").disabled = true;
    document.getElementById("timeoutTimerPause").disabled = false;

    var duration = timeoutTimeRemaining;
    elapsed_seconds = 0;

    var x = setInterval(function() {
        elapsed_seconds += 0.1;
        difference = duration - elapsed_seconds;

        var minutes = Math.floor(difference/60);
        var seconds = Math.floor(difference%60);

        document.getElementById("timeoutTimer").innerHTML = minutes + " m " + seconds + " s";

        // Timeout time can be paused by Pause and Reset Field buttons
        if(timeoutTimePaused){
            clearInterval(x);
            timeoutTimeRemaining = difference;
        }

        // Play sounds and update HTML components
        if (difference < 0) {
            horn.play();
            clearInterval(x);
            document.getElementById("timeoutTimer").innerHTML = "Expired";
            document.getElementById("timeoutTimerStart").disabled = true;
            document.getElementById("timeoutTimerPause").disabled = true;
        }else if (difference > (timeoutWarning-0.1) && difference < (timeoutWarning+0.1)) {
            bell.play();
        }
    }, 100);
    console.log("Started Timeout Timer!")
}

// onPauseTimeoutTimer()
// Pause button activates this function
// Updates timeoutTimePaused variable and updates HTML componenets 
function onPauseTimeoutTimer(){
    timeoutTimePaused = true;

    document.getElementById("timeoutTimerPause").disabled = true;
    document.getElementById("timeoutTimerStart").disabled = false;
    
}

// onSampleCollected()
// sample button activates this function
// Updates demoTimePaused, goalScored variables and updates HTML componenets 
function onSampleCollected(sampleType){
    // demoTimePaused = true;
    // goalScored = true;
    if(sampleType == 0){
        var goalTime = demoDuration - difference;
        var goalRate = demoTimeLastPausedAt - difference;
        updateSampleCollectedInfo(goalTime, goalRate, 0);
        videoGoal = true;
        // document.getElementById("goalButton").disabled = true;
        // document.getElementById("demoTimerPause").disabled = true;
        // document.getElementById("demoTimerStart").disabled = false;
    }else if(sampleType == 1){
        var goalTime = demoDuration - difference;
        var goalRate = demoTimeLastPausedAt - difference;
        updateSampleCollectedInfo(goalTime, goalRate, 1);
        videoGoal = true;
        // document.getElementById("goalButton").disabled = true;
        // document.getElementById("demoTimerPause").disabled = true;
        // document.getElementById("demoTimerStart").disabled = false;
    }
}

// updateGoalInfo()
// Displays goal time and rate 
function updateSampleCollectedInfo(sampleTime, sampleRate, sampleType){

    var openSampleList = document.getElementById("openSampleList")
    var rockSampleList = document.getElementById("rockSampleList")
    const sample_collected_string = document.createElement('li');
    
    const sample_ts = document.createElement('li');
    sample_ts.innerHTML = "Scored @ "+ Math.floor(sampleTime/60) + " m " + Math.floor(sampleTime%60) + " s";
    sample_ts.innerHTML += " | Rate: " + Math.floor(sampleRate/60) + " m " + Math.floor(sampleRate%60) + " s";
    const sampletime = document.createElement('ul');
    sampletime.appendChild(sample_ts);

    if(sampleType == 0){
        open_sample_score += 1; 
        sample_collected_string.innerHTML = "<strong>"+"Open Sample Collected "+open_sample_score+"</strong>";
        document.getElementById("open_sample_score").innerHTML = open_sample_score;

        openSampleList.appendChild(sample_collected_string);
        openSampleList.appendChild(sampletime);
    }else{
        rock_sample_score += 1; 
        sample_collected_string.innerHTML = "<strong>"+"Rock Sample Collected "+rock_sample_score+"</strong>";
        document.getElementById("rock_sample_score").innerHTML = rock_sample_score;

        rockSampleList.appendChild(sample_collected_string);
        rockSampleList.appendChild(sampletime);
    }
    


}

// updatePenaltyInfo()
// Displays penalty info
function updatePenaltyInfo(penaltyType){
    var penalties = document.getElementById("penaltyList")
    const penaltyItem = document.createElement('li');
    
    if(penaltyType == 0){
        penalty += 1;
        penaltyItem.innerHTML = "<strong>Collision</strong>";
    }

    document.getElementById("penalty").innerHTML = penalty;
    penalties.appendChild(penaltyItem);
}