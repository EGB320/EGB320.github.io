// Initialize variables
var fieldNo = 1;

var groupNo = 0;

// Setup Timing Variables
var setupTimePaused = false;
var setupDuration = 600;
var setupWarning = 60;

// Demo Timing Variables
var demoTimePaused = false;
var demoDuration = 300;
var demoTimeRemaining = 300;
var demoTimeLastPausedAt = 300;
var demoWarning = 60;

// Timeout Timing Variables
var timeoutTimePaused = false;
var timeoutTimeRemaining = 300;
var timeoutWarning = 60;

// Goal State Variables
var goalScored = false;
var videoGoal = false;
var open_sample_score = 0;
var rock_sample_score = 0;
// var total_sample_score = 0;

var penalty = 0;

// Update variables with the default values of the fields in settings tab
updateGamePlay(0);

// saveSettings()
// Save Changes button triggers this function
// Updates the variable values from settings form inputs
function saveSettings(){
    fieldNo = parseInt(document.getElementById("fieldNoText").value);

    groupNo = parseInt(document.getElementById("groupNoText").value);

    // Setup for timers
    setupDuration = parseInt(document.getElementById("setupDurationText").value);

    demoDuration = parseInt(document.getElementById("demoDurationText").value);
    demoTimeRemaining = parseInt(document.getElementById("demoDurationText").value);
    demoTimeLastPausedAt = parseInt(document.getElementById("demoDurationText").value);

    timeoutTimeRemaining = parseInt(document.getElementById("timeoutDurationText").value);

    //Warning time settings
    setupWarning = parseInt(document.getElementById("setupWarningText").value);
    demoWarning = parseInt(document.getElementById("demoWarningText").value);
    timeoutWarning = parseInt(document.getElementById("timeoutWarningText").value);

    updateGamePlay(1)
}

// updateGamePlay(call_type)
// Updates the HTML components using variable values
// call_type == 0 -> Call from page load or Reset Field button
// call_type == 1 -> Call from Save Changes button
function updateGamePlay(call_type){
    document.getElementById("demoInfo").innerHTML = "Field " + fieldNo + " | Group " + groupNo + " | ";
    
    document.getElementById("setupTimer").innerHTML = Math.floor(setupDuration/60) + " m " + Math.floor(setupDuration%60) + " s";
    document.getElementById("demoTimer").innerHTML = Math.floor(demoDuration/60) + " m " + Math.floor(demoDuration%60) + " s";
    document.getElementById("timeoutTimer").innerHTML = Math.floor(timeoutTimeRemaining/60) + " m " + Math.floor(timeoutTimeRemaining%60) + " s";

    document.getElementById("open_sample_score").innerHTML = open_sample_score;
    document.getElementById("rock_sample_score").innerHTML = rock_sample_score;
    // document.getElementById("total_sample_score").innerHTML = total_sample_score;
    document.getElementById("openSampleList").innerHTML = "";
    document.getElementById("rockSampleList").innerHTML = "";

    document.getElementById("penalty").innerHTML = penalty;
    document.getElementById("penaltyList").innerHTML = "";

    if(call_type == 1){
        document.getElementById("changesReturn").innerHTML = "&nbsp Saved Changes";
        setTimeout(function(){ 
            document.getElementById('nav-gameplay-tab').click();
            document.getElementById("changesReturn").innerHTML = "";
            document.getElementById("groupNoText").value = "";
        }, 1000);
    }

    document.getElementById("openSampleButton").disabled = true;
    document.getElementById("rockSampleButton").disabled = true;
    document.getElementById("penaltyButton").disabled = true;

    document.getElementById("setupTimerStart").disabled = false;
    document.getElementById("demoTimerPause").disabled = true;
    document.getElementById("demoTimerStart").disabled = true;
    document.getElementById("timeoutTimerPause").disabled = true;
    document.getElementById("timeoutTimerStart").disabled = true;

}

// resetField()
// Reset Field button calls this function to set every variable and HTML element to the their default values
function resetField(){
    // Stop all timers
    setupTimePaused = true;
    demoTimePaused = true;
    timeoutTimePaused = true;
    
    setTimeout(function(){ 
        // Set the variables to their default values
        groupNo = 0;

        setupTimePaused = false;
        setupDuration = 600;
        setupWarning = 60;

        demoTimePaused = false;
        demoDuration = 300;
        demoTimeRemaining = 300;
        demoTimeLastPausedAt = 300;
        demoWarning = 60;

        timeoutTimePaused = false;
        timeoutTimeRemaining = 300;
        timeoutWarning = 60;

        goalScored = false;
        videoGoal = false;
        open_sample_score = 0;
        rock_sample_score = 0;
        // total_sample_score = 0;

        penalty = 0;
    
        // Update Game Play tab
        updateGamePlay(0);

        // Cleans up the videos generates for the previous demo
        vids.forEach(function(url) {
            window.URL.revokeObjectURL(url);
        });
        vids = [];
        recordedVids = 1;
        document.getElementById('accordion').innerHTML = "";
        document.getElementById('recordedVideos').style = "display: none;";
        
        // Navigates to the settings tab
        document.getElementById('nav-settings-tab').click();
    }, 200);
}

// playSound(sound)
// Test Sounds buttons call this function to play game sounds
function playSound(sound){
    if(sound==0) whistle.play();
    if(sound==1) bell.play();
    if(sound==2) horn.play();
}