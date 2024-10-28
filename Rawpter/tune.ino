/**
 * Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
 */
void calculate_IMU_error() {
  /*
   * The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata().
   * this will correct for sensor drift and crookedness in your copter.
   */
  float AcX, AcY, AcZ, GyX, GyY, GyZ;

  Serial.println("Calculating IMU Error with 10000 iterations. Please stand by...");

  //Read IMU values 1000 times.  Should take around 2 minutes at 104Hz IMU timing.
  int c = 0;AccErrorX=0;AccErrorY=0;AccErrorZ=0;GyroErrorX=0;GyroErrorY=0;GyroErrorZ=0;
  while (c < 10000) {
    while (!IMU.accelerationAvailable()&!IMU.gyroscopeAvailable()) delay(1);
    IMU.readAcceleration(AcX, AcY, AcZ);
    IMU.readGyroscope(GyX, GyY, GyZ);
    AccX = AcX;
    AccY = AcY;
    AccZ = AcZ;
    GyroX = GyX;
    GyroY = GyY;
    GyroZ = GyZ;

    //Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
    Serial.println(String(10000-c));
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  AccErrorZ = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
  while (true) delay(1000);
}