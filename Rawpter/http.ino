#include <ArduinoJson.h>

// Define the pin for the built-in LED
#define LED_PIN LED_BUILTIN

void setupWiFi()
{
  char ssid[] = "_Rawpter";
  char pass[] = "12345678";

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("Please upgrade the firmware");
  }

  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING)
  {
    Serial.println("Creating access point failed");
    while (true)
      ;
  }

  // wait 1 seconds for connection:
  delay(1000);

  // start the web server on port 80
  server.begin();

  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with the LED off

  printWiFiStatus();
}

void printWiFiStatus()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void loopWiFi()
{
  WiFiClient client = server.available();
  if (client)
  {
    String request = "";
    unsigned long timeout = millis() + 5000; // 5 second timeout

    // Read the entire request
    while (client.connected() && millis() < timeout)
    {
      if (client.available())
      {
        char c = client.read();
        request += c;
        if (c == '\n' && request.endsWith("\r\n\r\n"))
        {
          // End of headers
          break;
        }
      }
    }

    // Process the request
    if (!request.isEmpty())
    {
      Serial.println("Received request:");
      Serial.println(request);
      handleRequest(client, request);
    }

    // Close the connection
    client.stop();
  }
}

void handleRequest(WiFiClient &client, const String &request)
{
  // Set CORS headers for all responses
  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS");
  client.println("Access-Control-Allow-Headers: Content-Type");

  // Handle CORS preflight request
  if (request.startsWith("OPTIONS"))
  {
    client.println("Content-Type: text/plain");
    client.println();
    return;
  }

  if (request.startsWith("GET /config"))
  {
    sendJSONResponse(client, createJSON());
  }
  else if (request.startsWith("POST /config"))
  {
    updateDataFromJSON(client);
  }
  else
  {
    // Handle unknown requests
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println();
    client.println("404 Not Found");
  }
}

void sendJSONResponse(WiFiClient &client, const String &data)
{
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.println(data);
}

String createJSON()
{
  StaticJsonDocument<512> doc;

  doc["roll_des"] = roll_des;
  doc["roll_IMU"] = roll_IMU;
  doc["pitch_des"] = pitch_des;
  doc["pitch_IMU"] = pitch_IMU;
  doc["loop_time"] = int(round(1 / deltaTime));
  doc["PWM_throttle"] = PWM_throttle;
  doc["battery_voltage"] = (4.4695 / 104.0 + ((float)batteryVoltage / 104.0));
  doc["maxMotor"] = maxMotor;
  doc["stick_dampener"] = stick_dampener;
  doc["i_limit"] = i_limit;
  doc["Accel_filter"] = Accel_filter;
  doc["Gyro_filter"] = Gyro_filter;
  doc["UPONLYMODE"] = UPONLYMODE;
  doc["Kp_roll_angle"] = Kp_roll_angle;
  doc["Ki_roll_angle"] = Ki_roll_angle;
  doc["Kd_roll_angle"] = Kd_roll_angle;
  doc["Kp_pitch_angle"] = Kp_pitch_angle;
  doc["Ki_pitch_angle"] = Ki_pitch_angle;
  doc["Kd_pitch_angle"] = Kd_pitch_angle;
  doc["Kp_yaw"] = Kp_yaw;
  doc["Ki_yaw"] = Ki_yaw;
  doc["Kd_yaw"] = Kd_yaw;
  doc["led_status"] = digitalRead(LED_PIN) == HIGH ? "on" : "off";

  String output;
  serializeJson(doc, output);
  return output;
}

void updateDataFromJSON(WiFiClient &client)
{
  // Skip headers
  while (client.available() && client.readStringUntil('\n') != "\r") {}

  // Read the body of the request
  String jsonStr = "";
  while (client.available()) {
    jsonStr += (char)client.read();
  }

  Serial.println("Received JSON:");
  Serial.println(jsonStr);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    sendJSONResponse(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }

  // Update variables
  if (doc.containsKey("maxMotor"))
    maxMotor = doc["maxMotor"];
  if (doc.containsKey("stick_dampener"))
    stick_dampener = doc["stick_dampener"];
  if (doc.containsKey("i_limit"))
    i_limit = doc["i_limit"];
  if (doc.containsKey("Accel_filter"))
    Accel_filter = doc["Accel_filter"];
  if (doc.containsKey("Gyro_filter"))
    Gyro_filter = doc["Gyro_filter"];
  if (doc.containsKey("UPONLYMODE"))
    UPONLYMODE = doc["UPONLYMODE"];
  if (doc.containsKey("Kp_roll_angle"))
    Kp_roll_angle = doc["Kp_roll_angle"];
  if (doc.containsKey("Ki_roll_angle"))
    Ki_roll_angle = doc["Ki_roll_angle"];
  if (doc.containsKey("Kd_roll_angle"))
    Kd_roll_angle = doc["Kd_roll_angle"];
  if (doc.containsKey("Kp_pitch_angle"))
    Kp_pitch_angle = doc["Kp_pitch_angle"];
  if (doc.containsKey("Ki_pitch_angle"))
    Ki_pitch_angle = doc["Ki_pitch_angle"];
  if (doc.containsKey("Kd_pitch_angle"))
    Kd_pitch_angle = doc["Kd_pitch_angle"];
  if (doc.containsKey("Kp_yaw"))
    Kp_yaw = doc["Kp_yaw"];
  if (doc.containsKey("Ki_yaw"))
    Ki_yaw = doc["Ki_yaw"];
  if (doc.containsKey("Kd_yaw"))
    Kd_yaw = doc["Kd_yaw"];
  if (doc.containsKey("led_status"))
  {
    if (doc["led_status"] == true)
      digitalWrite(LED_PIN, HIGH);
    else if (doc["led_status"] == false)
      digitalWrite(LED_PIN, LOW);
  }

  sendJSONResponse(client, "{\"status\":\"success\"}");
}
