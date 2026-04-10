void motors(int Motor1, int Motor2, int Motor3, int Motor4, bool reset)
{
  _log("move", String(Motor1) + String(Motor2) + String(Motor3) + String(Motor4));
  WRITE_MOTOR(M1, Motor1);
  WRITE_MOTOR(M2, Motor2);
  WRITE_MOTOR(M3, Motor3);
  WRITE_MOTOR(M4, Motor4);

  if(reset)
  {
    drive_m1 = 0;
    drive_m2 = 0;
    drive_m3 = 0;
    drive_m4 = 0;
  }

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
  //  _log("rotate to", String(error));
  }

  _log("rotate to", String(error) + " " + String(precision));

  if (abs(error) < precision)
  {
    return;
  }

  out = error * error * error;
  if (abs(error) < precision_switch)
  {
    //use precise speed
    out *= speed_precise;
  }else
  {
    //use general speed
    out *= speed;
    out += base_speed;
  }
  

  set_motors(out, out, out, out);
}

void rotate_to_quadratic(int target, int precision, int precision_switch, float speed, float speed_precise, int base_speed, int max_out = 10)
{
  int out;
  error = yaw - target;

  if (error >= 180) {
    error = -(360 - error);
    base_speed = -base_speed;
  }

  _log("rotate to", String(error) + " " + String(precision));

  if (abs(error) < precision) return;

  out = error * error;

  if (abs(error) < precision_switch) {
    out *= speed_precise;
    // Normalisieren: bei max error → max_out
    int max_raw = (int)(precision_switch * precision_switch * speed_precise);
    out = out * max_out / max(max_raw, 1);
  } else {
    out *= speed;
    out += base_speed;
    int max_raw = (int)(180 * 180 * speed) + abs(base_speed);
    out = out * max_out / max(max_raw, 1);
  }

  if (error < 0) out = -out;

  set_motors(out, out, out, out);
}

void rotate_quadratic(int angle, int precision, int precision_switch, int max_out, int min_out)
{
  error = angle;
  if (error >= 180) error = -(360 - error);
  if (abs(error) < precision) return;

  int out;
  if (abs(error) < precision_switch) {
    // quadratisch normalisiert auf precision_switch
    out = max_out * ((float)(error * error) / (float)(precision_switch * precision_switch));
  } else {
    // quadratisch normalisiert auf 90°
    out = max_out * ((float)(error * error) / (float)(90 * 90));
    if (out > max_out) out = max_out;
  }

  if (out < min_out) out = min_out; // Mindestgeschwindigkeit damit er sich immer bewegt
  if (error < 0) out = -out;

  set_motors(out, out, out, out);
}

void rotate(int speed)
{
  set_motors(speed, speed, speed, speed);
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
  if ( !(abs(error) < angle_precision) )
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

void orbit_around(int target, int Speed, int orbit_radius)
{
  error = yaw - target;
  if (error >= 180) {
    error = -(360 - error);
  }

  int YMotors = cos((target - 45) * (M_PI / 180.0)) * Speed;
  int XMotors = sin((target - 45) * (M_PI / 180.0)) * Speed;

  int orbit_correction = error * orbit_radius;

  int m1 =  YMotors + orbit_correction;
  int m2 = -XMotors + orbit_correction;
  int m3 = -YMotors + orbit_correction;
  int m4 =  XMotors + orbit_correction;

  // Größten Betrag finden
  int max_val = max({abs(m1), abs(m2), abs(m3), abs(m4)});

  // Normalisieren auf Speed, nur wenn max_val > 0
  if (max_val > 0) {
    m1 = m1 * Speed / max_val;
    m2 = m2 * Speed / max_val;
    m3 = m3 * Speed / max_val;
    m4 = m4 * Speed / max_val;
  }

  set_motors(m1, m2, m3, m4);
}
void orbit(int speed, int ang, int spe)
{
  move_angle(ang, speed);
  rotate(spe);
}

