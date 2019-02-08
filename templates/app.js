var startButton = $("#main");
var calibrateButton = $("#calibrate");
var homeButton = $("#home");

var states = {
  unengaged: "unengaged",
  engaged: "engaged",
  calibrating: "calibrating",
  uncalibrated: "uncalibrated",
};

var state = states.unengaged;

function startstop(){
  calibrateButton.toggleClass("disabled");
  homeButton.toggleClass("disabled");

  if (state == states.unengaged) {
    state = states.engaged;
    startButton.html("Stop Arm Script");
    fetch("/main").then(resp => console.log("Started script"));
  }
  else {
    state = states.unengaged;
    startButton.html("Start Arm Script");
    fetch("/stopmain").then(resp => console.log("Stopped script"));
  }
}

function calibrate() {
  startButton.toggleClass("disabled");
  homeButton.toggleClass("disabled");

  if (state != states.calibrating) {
    state = states.calibrating;
    calibrateButton.html("Arm is in Calibrate Position?");
    fetch("/learning").then(resp => console.log("In learning mode"));
  }
  else if (state == states.calibrating) {
    state = states.unengaged;
    calibrateButton.html("Calibrate Arm");
    fetch("/calibrate").then(resp => console.log("Calibration complete"));
  }
}

function home() {
  fetch("/crouchingtiger").then(resp => console.log(":)"));
}

function em(status) {
  fetch(`/em/${status}`).then(resp => console.log(`em ${status}`));
}