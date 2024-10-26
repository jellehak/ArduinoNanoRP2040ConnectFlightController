void setupWiFi() {
  char ssid[] = "_Rawpter";
  char pass[] = "12345678";

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue because this is probably not an RP2040
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));
  // Just picked this out of the air.  Throw back to Jeff Gordon
  // WiFi.config(IPAddress(192, 168, 2, 4));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }

  // wait 1 seconds for connection:
  delay(1000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void loopWiFi() {
  // compare the previous status to the current status
  WiFiClient client = server.available();  // listen for incoming clients
  if (client) {                            // if you get a client,
    String currentLine = "";               // make a String to hold incoming data from the client
    while (client.connected()) {           // loop while the client's connected
      if (client.available()) {            // if there's bytes to read from the client,
        char c = client.read();            // read a byte, then
        if (c == '\n') {
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        // if (currentLine.endsWith("OPTIONS /data.json")) {
        //   // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
        //   // and a content-type so the client knows what's coming, then a blank line:
        //   client.println("HTTP/1.1 200 OK");
        //   client.println("Access-Control-Allow-Origin: *");
        //   // client.println("Content-type:application/json");
        //   client.println();
        //   // the content of the HTTP response follows the header:
        //   client.print(data);
        //   client.println();  // The HTTP response ends with another blank line:
        //   continue;
        // }
// Serial.println(currentLine);

        if (currentLine.endsWith("GET /data.json")) {
          String data = createJSON();

          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:application/json");
          client.println("Access-Control-Allow-Origin: *");
          client.println();

          // the content of the HTTP response follows the header:
          client.print(data);
          client.println();  // The HTTP response ends with another blank line:

          continue;
        }

        if (currentLine.endsWith("GET /?")) {
          setTheValuesFromUserForm(client);
          //Handle clicking the Begin button
          MakeWebPage(client, "<meta http-equiv = \"refresh\" content = \"0; url = http://192.168.2.4 \"/>");
        } else if (currentLine.endsWith("GET / HTTP")) {  //Handle hitting the basic page (1st connection)
          String myMsg = "<h1>Rawpter V1.7</h1><small>by Raising Awesome</small><br>";
          //For the battery voltage calc, you can use ohm's law on your chosen voltage divider resistors and get the voltage ratio of the 12bit ADC.  I simply recorded values against fed voltages from my bench power supply and fit 
          //a line.  Good ole' y=mx+b.
          myMsg += "<b>Snapshot:</b><br>Desired Roll=" + String(roll_des) + "&#176;&nbsp;&nbsp;&nbsp;IMU Roll=" + String(roll_IMU) + "&#176;<br>Desired Pitch= " + String(pitch_des) + "&#176;&nbsp;&nbsp;&nbsp;IMU Pitch=" + String(pitch_IMU) + "&#176;<br>Loop Time= " + String(int(round(1 / (deltaTime)))) + "&nbsp;&nbsp;&nbsp;Throttle PWM=" + String(PWM_throttle) + "<br>Battery=" + String( (4.4695/104.0+((float)batteryVoltage/104.0)),1) + "V (" + String(batteryVoltage) + ")<br>";
          if (batteryVoltage<1350) myMsg+="<br><div class='alert alert-danger'>DANGER: BATTERY LOW!</div><br>";
          myMsg += GetParameters() + "<br><input class='mt-2 btn btn-primary' type=submit value='submit' />";
          MakeWebPage(client, myMsg);
        }
      }
    }
    // close the connection:
    client.stop();
  }
}

void setTheValuesFromUserForm(WiFiClient client) {
  String currentLine = "";
  UPONLYMODE = false;
  while (client.connected()) {  // loop while the client's connected
    if (client.available()) {   // if there's bytes to read from the client,
      char c = client.read();   // read a byte, then
      if (c == '\n') {
        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          break;
        } else {  // if you got a newline, then clear currentLine:
          currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }
      if (currentLine.endsWith("maxMotor=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        maxMotor = myValue.toFloat();
      }
      if (currentLine.endsWith("stick_dampener=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        stick_dampener = myValue.toFloat();
      }

      if (currentLine.endsWith("i_limit=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        i_limit = myValue.toFloat();
      }
      if (currentLine.endsWith("Accel_filter=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Accel_filter = myValue.toFloat();
      }
      if (currentLine.endsWith("Gyro_filter=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Gyro_filter = myValue.toFloat();
      }

      if (currentLine.endsWith("UPONLYMODE=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        if (myValue == "true") UPONLYMODE = true;
        else UPONLYMODE = false;
      }

      if (currentLine.endsWith("kp_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_roll_angle = myValue.toFloat();
      }

      if (currentLine.endsWith("ki_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_roll_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_roll_angle = myValue.toFloat();
      }

      //
      if (currentLine.endsWith("kp_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_pitch_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("ki_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_pitch_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_pitch_angle = myValue.toFloat();
      }
      //yaw
      if (currentLine.endsWith("kp_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_yaw = myValue.toFloat();
      }
      if (currentLine.endsWith("ki_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_yaw = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_yaw = myValue.toFloat();
        return;
      }
    }
  }
}

String GetParameters() {
  String myString;
  myString = myString + "<table><tr><td>Max Motor Speed (0.0-1.0):</td><td><input type=number step=.001 name=maxMotor style='width:70px;' value='" + String(maxMotor) + "'></td></tr>";
  myString = myString + "<tr><td>Stick Dampening (0.01-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=stick_dampener style='width:70px;' value='" + String(stick_dampener) + "'></td></tr>";
  myString = myString + "<tr><td>Integral Accumulation (25 default):</td><td><input type=number step=.001 name=i_limit style='width:70px;' value='" + String(i_limit) + "'></td></tr>";
  myString = myString + "<tr><td>Accel Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=Accel_filter style='width:70px;' value='" + String(Accel_filter) + "'></td></tr>";
  myString = myString + "<tr><td>Gyro Dampening (0.1-1.0 ):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=Gyro_filter style='width:70px;' value='" + String(Gyro_filter) + "'></td></tr>";
  String myVal;
  if (!UPONLYMODE) myVal = "";
  else myVal = "checked";
  myString = myString + "<tr><td>Up Mode Only:</td><td><input type=CHECKBOX name=UPONLYMODE value='true' " + (myVal) + "></td></tr>";
  myString = myString + "</table>";
  myString = myString + "<table class=table><thead class=thead-dark><th></th><th>Kp</th><th>Ki</th><th>Kd</th></thead>";
  myString = myString + "<tr><td>Roll:</td><td><input name=kp_roll_angle style='width:70px;' type=number step=.001 value='" + String(Kp_roll_angle) + "'></td><td><input style='width:70px;' name=ki_roll_angle type=number step=.001 value='" + String(Ki_roll_angle) + "'></td><td><input name=kd_roll_angle type=number step=.001 style='width:70px;' value='" + String(Kd_roll_angle) + "'></td></tr>";
  myString = myString + "<tr><td>Pitch:</td><td><input name=kp_pitch_angle  style='width:70px;' type=number step=.001 value='" + String(Kp_pitch_angle) + "'></td><td><input  style='width:70px;' name=ki_pitch_angle type=number step=.001 value='" + String(Ki_pitch_angle) + "'></td><td><input name=kd_pitch_angle type=number step=.001 style='width:70px;' value='" + String(Kd_pitch_angle) + "'></td></tr>";
  myString = myString + "<tr><td>Yaw:</td><td><input name=kp_yaw type=number step=.001 style='width:70px;' value='" + String(Kp_yaw) + "'></td><td><input  style='width:70px;' type=number step=.001 name=ki_yaw value='" + String(Ki_yaw) + "'></td><td><input type=number step=.001 name=kd_yaw style='width:70px;' value='" + String(Kd_yaw) + "'></td></tr><input type=hidden name=ender value='0'>";
  myString = myString + "</table><br>Tip: To use your parameters beyond this flight session, snapshot the screen for reference and update the code.<br>";
  return myString;
}

void MakeWebPage(WiFiClient client, String html) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();

  // the content of the HTTP response follows the header:
  client.print("<head>");
  client.print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.print("<link href=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css\" rel=\"stylesheet\" integrity=\"sha384-1BmE4kWBq78iYhFldvKuhfTAU6auU8tT94WrHftjDbrCEXSU1oBoqyl2QvZ6jIW3\" crossorigin=\"anonymous\">");
  client.print("</head>");
  client.print("<style>");
  client.print("</style>");
  client.print("<form method=get><div class='container'>");
  client.print(html);
  client.print(tuningProcedure());
  client.print("</div></form>");
  client.print("<script src=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js\" integrity=\"sha384-ka7Sk0Gln4gmtz2MlQnikT1wXgYsOg+OMhuP+IlRH9sENBO0LRn5q+8nbTov4+1p\" crossorigin=\"anonymous\"></script>");
  client.println();  // The HTTP response ends with another blank line:
}