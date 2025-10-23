void motors(int Motor1, int Motor2, int Motor3, int Motor4)
{
  _log("move", String(Motor1) + String(Motor2) + String(Motor3) + String(Motor4));
  WRITE_MOTOR(M1, Motor1);
  WRITE_MOTOR(M2, Motor2);
  WRITE_MOTOR(M3, Motor3);
  WRITE_MOTOR(M4, Motor4);
}

void set_motors(int Motor1, int Motor2, int Motor3, int Motor4)
{
  _log("set_move", String(Motor1) + String(Motor2) + String(Motor3) + String(Motor4));
  drive_m1 += Motor1;
  drive_m2 += Motor2;
  drive_m3 += Motor3;
  drive_m4 += Motor4;
}

void rotate_to(int target, int speed, int base_speed)
{
  _imu_read();

  error = target - yaw;
  _log("rotoa", String(error));
  if (error < 0) {error += 180;}

  error *= speed;
  error += base_speed;

  set_motors(error, error, error, error);
}

void move_angle(int target, int Speed)
{
  // Y 1,3; cos(target - 45)
  int YMotors= cos( (target) * (M_PI / 180.0) ) * Speed ;

  // X 2,4; sin(target - 45)
  int XMotors = sin( (target) * (M_PI / 180.0) ) * Speed ;

  set_motors(YMotors, -XMotors, -YMotors, XMotors);
}