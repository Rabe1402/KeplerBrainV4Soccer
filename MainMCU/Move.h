void motors(int Motor1, int Motor2, int Motor3, int Motor4)
{
  _log("move", String(Motor1) + String(Motor2) + String(Motor3) + String(Motor4));
  WRITE_MOTOR(M1, Motor1);
  WRITE_MOTOR(M2, Motor2);
  WRITE_MOTOR(M3, Motor3);
  WRITE_MOTOR(M4, Motor4);

  drive_m1 = 0;
  drive_m2 = 0;
  drive_m3 = 0;
  drive_m4 = 0;

}

void set_motors(int Motor1, int Motor2, int Motor3, int Motor4)
{
  _log("set_move", String(Motor1) + String(Motor2) + String(Motor3) + String(Motor4));
  drive_m1 += Motor1;
  drive_m2 += Motor2;
  drive_m3 += Motor3;
  drive_m4 += Motor4;

}

void rotate_to(int target, int precision, int precision_switch,  float speed, float speed_precise, int base_speed)
{
//  _imu_read();
  int out;
  error = yaw - target;

  if (error >= 180) {
    error = -(360 - error);
    base_speed = -base_speed;
  //  _log("rotate to 2", String(error));
  }

  _log("rotate to", String(error) + " " + String(precision));

  if (abs(error) < precision)
  {
    return;
  }

  if (abs(error) < precision_switch)
  {
    //use precise speed
    out = error * error * error;
    out *= speed_precise;
  }else
  {
    //use general speed
    out = error * error * error;
    out *= speed;
    out += base_speed;
  }
  

  set_motors(out, out, out, out);
}

void move_angle(int target, int Speed)
{
  // Y 1,3; cos(target - 45)
  int YMotors= cos( (target - 45) * (M_PI / 180.0) ) * Speed ;

  // X 2,4; sin(target - 45)
  int XMotors = sin( (target - 45) * (M_PI / 180.0) ) * Speed ;

  set_motors(YMotors, -XMotors, -YMotors, XMotors);
}

void move_angle_correction(int target, int Speed, int angle_precision)
{
  //  _imu_read();

  error = yaw - target;
  if (! error < angle_precision)
  {

    _log("move angle correction", String(error));

    if (error >= 180) {

      error = -(360 - error);

    }
  }

  target += error;

  // Y 1,3; cos(target - 45)
  int YMotors= cos( (target + 45) * (M_PI / 180.0) ) * Speed ;

  // X 2,4; sin(target - 45)
  int XMotors = sin( (target + 45) * (M_PI / 180.0) ) * Speed ;

  set_motors(YMotors, -XMotors, -YMotors, XMotors);
}