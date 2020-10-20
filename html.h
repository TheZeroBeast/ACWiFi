const char INDEX_page[] PROGMEM = R"=====(
<html>
<head>
  <style>
    html {
	  font-family: Arial;
	  display: inline-block;
	  margin: 0px auto;
	  text-align: center;
	}
	h1 {
	  color: #0F3376;
	  padding: 2vh;
	}
	p {
	  font-size: 1.5rem;
	}
	.boxed {
	  border: 1px solid black;
	  margin-left: auto;
	  margin-right: auto;
	  width: 500px;
	}
	.setup {
	}
	.button {
	  display: inline-block;
	  background-color: #008CBA;
	  border: none;
	  border-radius: 4px;
	  color: white;
	  padding: 16px 40px;
	  text-decoration: none;
	  font-size: 30px;
	  margin: 2px;
	  cursor: pointer;
	}
	.button2 {
	  background-color: #f44336;
	}
	.button3 {
	  background-color: #800080;
	}
	.button4 {
	  background-color: #11DBBC;
	}
	.button5 {
	  background-color: #F11F85;
	}
	.button6 {
	  background-color: #15CA70;
	}
	.button7 {
	  background-color: #9611F8;
	}
	.units {
	  font-size: 1.2rem;
	 }
	.sensor-labels {
	  font-size: 1.5rem;
	  vertical-align:middle;
	  padding-bottom: 15px;
	}
	.submit-button {
	  font-size: 1.5rem;
	  vertical-align:middle;
	}
	.center {
	  display: block;
	  margin-left: auto;
	  margin-right: auto;
	  width: 30%;
	}
  </style>
  <title>AC WiFi v1.0</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <link rel="stylesheet" type="text/css" href="style.css">
</head>
<body> 
  <h1>AC WiFi v1.0</h1>
  <h2>Remote Control</h2>
  <p>
      <h3>Power</h3>
      <a href="/poweron"><button class="button">ON</button></a>
      <a href="/poweroff"><button class="button button2">OFF</button></a>
	  <h3>Mode</h3>
	  <a href="/modecool"><button class="button">COOL</button></a>
	  <a href="/modedry"><button class="button button4">DRY</button></a>
      <a href="/modeheating"><button class="button button5">HEATING</button></a>
	  <a href="/modefan"><button class="button button6">FAN</button></a>
	  <a href="/modeauto"><button class="button button7">AUTO</button></a>
	  <h3>Set Temperature</h3>
	  <label class="sensor-labels" for="settemp">Current Setting: </label><span class="sensor-labels" id="currenttempsetting">%CURRENTTEMPSETTING%</span><sup class="units">&deg;C</sup><br>
	  <label class="sensor-labels" for="settemp">New Setting: </label><input class="submit-button" type="text" id="settemp" name="settemp" size="3"><input class="submit-button" type="submit" value="Set">
	  <h3>Fan Speed</h3>
	  <a href="/fanspeed1"><button class="button button6">1</button></a>
	  <a href="/fanspeed2"><button class="button button6">2</button></a>
	  <a href="/fanspeed3"><button class="button button6">3</button></a>
	  <a href="/fanspeed4"><button class="button button6">4</button></a>
  </p>
  <h2>Sensor Data</h2>
  <p>
    <span class="sensor-labels">Room Temperature</span>
    <span id="temperature">%TEMPERATURE%</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
	<a href="setup.html"><button class="button button3" type="button">Setup</button></a>
  </p>
</body>
<script>
  setInterval(function ( ) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("temperature").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "/temperature", true);
    xhttp.send();
  }, 1000 ) ;
  
  setInterval(function ( ) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("currenttempsetting").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "/currenttempsetting", true);
    xhttp.send();
  }, 1000 ) ;
</script>
</html>
)=====";

const char SETUP_page[] PROGMEM = R"=====(
<html>
<head>
  <style>
    html {
	  font-family: Arial;
	  display: inline-block;
	  margin: 0px auto;
	  text-align: center;
	}
	h1 {
	  color: #0F3376;
	  padding: 2vh;
	}
	p {
	  font-size: 1.5rem;
	}
	.boxed {
	  border: 1px solid black;
	  margin-left: auto;
	  margin-right: auto;
	  width: 500px;
	}
	.setup {
	}
	.button {
	  display: inline-block;
	  background-color: #008CBA;
	  border: none;
	  border-radius: 4px;
	  color: white;
	  padding: 16px 40px;
	  text-decoration: none;
	  font-size: 30px;
	  margin: 2px;
	  cursor: pointer;
	}
	.button2 {
	  background-color: #f44336;
	}
	.button3 {
	  background-color: #800080;
	}
	.button4 {
	  background-color: #11DBBC;
	}
	.button5 {
	  background-color: #F11F85;
	}
	.button6 {
	  background-color: #15CA70;
	}
	.button7 {
	  background-color: #9611F8;
	}
	.units {
	  font-size: 1.2rem;
	 }
	.sensor-labels {
	  font-size: 1.5rem;
	  vertical-align:middle;
	  padding-bottom: 15px;
	}
	.submit-button {
	  font-size: 1.5rem;
	  vertical-align:middle;
	}
	.center {
	  display: block;
	  margin-left: auto;
	  margin-right: auto;
	  width: 30%;
	}
  </style>
  <title>Setup</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <link rel="stylesheet" type="text/css" href="style.css">
</head>
<body> 
  <h1>Setup</h1>
	<div class="setup">
		<p>	
			<h3>WiFi Setup</h3>
			<form action="/updatewifi">
			<label for="ssid">SSID:</label>
			<input type="text" id="ssid" name="ssid" size="20"><br><br>
			<label for="passphrase">PASS:</label>
			<input type="text" id="pass" name="pass" size="20"><br><br>
			<input type="submit" value="Submit">
			</form>
		</p>
		<p>
			<a href="/"><button class="button button3" type="button">Back</button></a>
		</p>
	</div>
</body>
</html>
)=====";