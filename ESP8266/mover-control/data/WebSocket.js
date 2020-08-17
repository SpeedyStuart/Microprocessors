var systemOn = false;
var connection = new WebSocket('ws://' + location.hostname + ':81/', ['arduino']);
connection.onopen = function () {
    connection.send('Connect ' + new Date());
};
connection.onerror = function (error) {
    console.log('WebSocket Error ', error);
};
connection.onmessage = function (e) {  
    console.log('Server: ', e.data);
};
connection.onclose = function(){
    console.log('WebSocket connection closed');
};

var joyParam = {
    title: "joystick",
    width: 200,
    height: 200,
    internalFillColor: "#19647E",
    internalStrokeColor: "#19647E",
    externalStrokeColor: "#1D1A05"
};
var Joy = new JoyStick('joystick', joyParam);

setInterval(function () {
    var data;
    if (systemOn) {
        var x = Joy.GetX();
        var y = Joy.GetY();
        data = "1" + x.toString() + "," + y.toString();
    } else {
        data = "0";
    }

    connection.send(data);    
}, 50);

function togglePower() {
    systemOn = !systemOn;
    setButton();
}

function setButton() {
    if (systemOn) {
        document.getElementById('powerButton').className = "button btn-primary on";
        document.getElementById('powerButton').value = "On";
    } else {
        document.getElementById('powerButton').className = "button btn-primary off";
        document.getElementById('powerButton').value = "Off";
    }
}
