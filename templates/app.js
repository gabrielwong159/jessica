var url = "http://10.12.175.231:1234";

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


function get(url, depth = 0) {
  if (depth == 10) return;

  return fetch(url)
    .catch(err => get(url, depth+1));
}

async function startstop(){
  if (state == states.unengaged) {
    let res = await get(url + "/main");
    if (res) console.log(await res.text());
    else return;

    state = states.engaged;
    startButton.html("Stop Arm Script");
  }
  else {
    let res = await get(url + "/stopmain");
    if (res) console.log(await res.text());
    else return;

    state = states.unengaged;
    startButton.html("Start Arm Script");
  }

  calibrateButton.toggleClass("disabled");
  homeButton.toggleClass("disabled");
}

async function calibrate() {
  calibrateButton.toggleClass("disabled");

  if (state != states.calibrating) {
    let res = await get(url + "/learning");
    if (res) console.log(await res.text());

    state = states.calibrating;
    calibrateButton.html("Arm is in Calibrate Position?");
  }
  else if (state == states.calibrating) {
    let res = await get(url + "/calibrate");
    if (res) console.log(await res.text());

    state = states.unengaged;
    calibrateButton.html("Calibrate Arm");
  }

  startButton.toggleClass("disabled");
  homeButton.toggleClass("disabled");
  calibrateButton.toggleClass("disabled");
}

async function home() {
  let res = await get(url + "/crouchingtiger");
  if (res) console.log(await res.text());
}

async function em(status) {
  let res = await get(url + `/em/${status}`);
  if (res) console.log(await res.text());
}
