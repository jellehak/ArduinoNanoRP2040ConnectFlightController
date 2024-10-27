void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(getRadioPWM(1));
    Serial.print(F(" CH2: "));
    Serial.print(getRadioPWM(2));
    Serial.print(F(" CH3: "));
    Serial.print(getRadioPWM(3));
    Serial.print(F(" CH4: "));
    Serial.println(getRadioPWM(4));
    Serial.print(F(" CH5: "));
    Serial.print(PWM_ThrottleCutSwitch);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}

void printJSON() {
  if (current_time - print_counter < 34000) return;  //Don't go too fast or it slows down the main loop

  print_counter = micros();
  Serial.print(F("{\"roll\": "));
  Serial.print(roll_IMU);
  Serial.print(F(", \"pitch\": "));
  Serial.print(pitch_IMU);
  Serial.print(F(", \"yaw\": "));
  Serial.print(yaw_IMU);
  Serial.print(F(", \"ErrorRoll\": "));
  Serial.print(error_roll);
  Serial.print(F(", \"IntegralRoll\": "));
  Serial.print(integral_roll);
  Serial.print(F(", \"DerivativeRoll\": "));
  Serial.print(derivative_roll);

  Serial.print(F(", \"RollKp\": "));
  Serial.print(Kp_roll_angle);
  Serial.print(F(", \"RollKi\": "));
  Serial.print(Ki_roll_angle);
  Serial.print(F(", \"RollKd\": "));
  Serial.print(Kd_roll_angle);

  Serial.print(F(", \"m1\": "));
  Serial.print(m1_command_PWM);
  Serial.print(F(", \"m2\": "));
  Serial.print(m2_command_PWM);
  Serial.print(F(", \"m3\": "));
  Serial.print(m3_command_PWM);
  Serial.print(F(", \"m4\": "));
  Serial.print(m4_command_PWM);

  Serial.print(F(", \"AccX\": "));
  Serial.print(AccX);
  Serial.print(F(", \"AccY\": "));
  Serial.print(AccY);
  Serial.print(F(", \"AccZ\": "));
  Serial.print(AccZ);

  Serial.print(F(", \"GyroX\": "));
  Serial.print(GyroX);
  Serial.print(F(", \"GyroY\": "));
  Serial.print(GyroY);
  Serial.print(F(", \"GyroZ\": "));
  Serial.print(GyroZ);
  Serial.print(F(", \"RollIMU\": "));
  Serial.print(roll_IMU);
  Serial.print(F(", \"PitchIMU\": "));
  Serial.print(pitch_IMU);
  Serial.print(F(", \"YawIMU\": "));
  Serial.print(yaw_IMU);

  Serial.print(F(", \"ThroDes\": "));
  Serial.print(thro_des);
  Serial.print(F(", \"RollDes\": "));
  Serial.print(roll_des);
  Serial.print(F(", \"PitchDes\": "));
  Serial.print(pitch_des);
  Serial.print(F(", \"YawDes\": "));
  Serial.print(yaw_des);
  Serial.print(F(", \"Pitch_PID\": "));
  Serial.print(pitch_PID);
  Serial.print(F(", \"Roll_PID\": "));
  Serial.print(roll_PID);
  Serial.print(F(", \"Yaw_PID\": "));
  Serial.print(yaw_PID);

  Serial.print(F(", \"PWM_throttle\": "));
  Serial.print(PWM_throttle);
  Serial.print(F(", \"PWM_roll\": "));
  Serial.print(PWM_roll);
  Serial.print(F(", \"PWM_Elevation\": "));
  Serial.print(PWM_Elevation);
  Serial.print(F(", \"PWM_Rudd\": "));
  Serial.print(PWM_Rudd);
  Serial.print(F(", \"PWM_ThrottleCutSwitch\": "));
  Serial.print(PWM_ThrottleCutSwitch);
  Serial.print(F(", \"Throttle_is_Cut\": "));
  Serial.print(throttle_is_cut);

  Serial.print(F(", \"Failsafe\": "));
  Serial.print(failsafed);

  Serial.print(F(", \"DeltaTime\": "));
  Serial.print(deltaTime * 1000000.0);
  Serial.println("}");
}


String createJSONOld() {
  String string;
  string.concat(F("{\"roll\": "));
  string.concat(roll_IMU);
  string.concat(F(", \"pitch\": "));
  string.concat(pitch_IMU);
  string.concat(F(", \"yaw\": "));
  string.concat(yaw_IMU);
  string.concat(F(", \"ErrorRoll\": "));
  string.concat(error_roll);
  string.concat(F(", \"IntegralRoll\": "));
  string.concat(integral_roll);
  string.concat(F(", \"DerivativeRoll\": "));
  string.concat(derivative_roll);

  string.concat(F(", \"RollKp\": "));
  string.concat(Kp_roll_angle);
  string.concat(F(", \"RollKi\": "));
  string.concat(Ki_roll_angle);
  string.concat(F(", \"RollKd\": "));
  string.concat(Kd_roll_angle);

  string.concat(F(", \"m1\": "));
  string.concat(m1_command_PWM);
  string.concat(F(", \"m2\": "));
  string.concat(m2_command_PWM);
  string.concat(F(", \"m3\": "));
  string.concat(m3_command_PWM);
  string.concat(F(", \"m4\": "));
  string.concat(m4_command_PWM);

  string.concat(F(", \"AccX\": "));
  string.concat(AccX);
  string.concat(F(", \"AccY\": "));
  string.concat(AccY);
  string.concat(F(", \"AccZ\": "));
  string.concat(AccZ);

  string.concat(F(", \"GyroX\": "));
  string.concat(GyroX);
  string.concat(F(", \"GyroY\": "));
  string.concat(GyroY);
  string.concat(F(", \"GyroZ\": "));
  string.concat(GyroZ);
  string.concat(F(", \"RollIMU\": "));
  string.concat(roll_IMU);
  string.concat(F(", \"PitchIMU\": "));
  string.concat(pitch_IMU);
  string.concat(F(", \"YawIMU\": "));
  string.concat(yaw_IMU);

  string.concat(F(", \"ThroDes\": "));
  string.concat(thro_des);
  string.concat(F(", \"RollDes\": "));
  string.concat(roll_des);
  string.concat(F(", \"PitchDes\": "));
  string.concat(pitch_des);
  string.concat(F(", \"YawDes\": "));
  string.concat(yaw_des);
  string.concat(F(", \"Pitch_PID\": "));
  string.concat(pitch_PID);
  string.concat(F(", \"Roll_PID\": "));
  string.concat(roll_PID);
  string.concat(F(", \"Yaw_PID\": "));
  string.concat(yaw_PID);

  string.concat(F(", \"PWM_throttle\": "));
  string.concat(PWM_throttle);
  string.concat(F(", \"PWM_roll\": "));
  string.concat(PWM_roll);
  string.concat(F(", \"PWM_Elevation\": "));
  string.concat(PWM_Elevation);
  string.concat(F(", \"PWM_Rudd\": "));
  string.concat(PWM_Rudd);
  string.concat(F(", \"PWM_ThrottleCutSwitch\": "));
  string.concat(PWM_ThrottleCutSwitch);
  string.concat(F(", \"Throttle_is_Cut\": "));
  string.concat(throttle_is_cut);

  string.concat(F(", \"Failsafe\": "));
  string.concat(failsafed);

  string.concat(F(", \"DeltaTime\": "));
  string.concat(deltaTime * 1000000.0);
  string.concat("}");
  return string;
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.println(m4_command_PWM);
  }
}

void printtock() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("deltaTime = "));
    Serial.println(deltaTime * 1000000.0);
  }
}