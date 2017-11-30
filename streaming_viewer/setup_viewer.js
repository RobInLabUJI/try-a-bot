var view = null;
var ipInput = null;
var portInput = null;
var connectButton = null;

function init() {
    view = new webots.View(document.getElementById("playerDiv"), document.getElementById("logDiv"));
    connectButton = document.getElementById("ConnectButton");
    $('body').layout({
        center__maskContents: true,
        south__size: 128,
        north__resizable: false
    });
}

function connect() {
    view.open("ws://" + location.hostname + ":1234");
     
    connectButton.value = "Disconnect";
    connectButton.onclick = disconnect;
    ipInput.disabled = true;
    portInput.disabled = true;
}

function disconnect() {
    view.close();

    connectButton.value = "Connect";
    connectButton.onclick = connect;
    ipInput.disabled = false;
    portInput.disabled = false;
}

window.addEventListener("load", init, false);
