// Stepper-Driver in 1/2-Mode

#include <Arduino.h>
#include "RobotArmStepperLib.h"

/************************************************************************************************************************
 *  Fahren eines Schrittmotors
 ************************************************************************************************************************/

void move_motor(int dir_pin, int step_pin, bool dir, int steps, int delay)
{
  digitalWrite(dir_pin, dir);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(delay);

    digitalWrite(step_pin, LOW);
    delayMicroseconds(delay);
  }
}

/************************************************************************************************************************
 *  nacheinander alle drei Schrittmotoren bis zu den jeweiligen Endschaltern fahren
 ************************************************************************************************************************/

void homing(const int x_step, const int x_dir, const int y_step, const int y_dir, const int z_step,
            const int z_dir, const int enPin, const int x_homePin, const int y_homePin, const int z_homePin)
{
  bool x_home = digitalRead(x_homePin);
  while (x_home == 1)
  {
    x_home = digitalRead(x_homePin);
    move_motor(x_dir, x_step, 1, 1, 3000);
  }
  delay(500);

  bool y_home = digitalRead(y_homePin);
  while (y_home == 1)
  {
    y_home = digitalRead(y_homePin);
    move_motor(y_dir, y_step, 0, 1, 3000);
  }
  delay(500);

  bool z_home = digitalRead(z_homePin);
  while (z_home == 1)
  {
    z_home = digitalRead(z_homePin);
    move_motor(z_dir, z_step, 0, 1, 2000);
  }
}

/*************************************************************************************************************************************************
 *    Nachfolgendes bezieht sich nur auf das Dreick das die beiden Greifer-Arme bilden und nicht auf den 3D-Raum.
 *    Wenn der Greifer exakt in Richtung der y-Achse zeigt und somit y = 0, können die y- und z-Werte auch auf der 3D-Raum übernommen werden
 *************************************************************************************************************************************************

    z-Achse
        |
        |       C
        |      /\
        |   b /  \ a     / Greiferzange                  Punkt A liegt nicht im Ursprung, da Versatz durch Befestigung der Motoren am Greifer 
  Offset|____/    \____/___                              Winkel Alpha und Beta sind immerauf y-Achse bezogen (wegen Kalibrierung und Referenz) 
        |   |A  c  B
        |   |
        0 --|------------------- y-Achse 
           Offset


*/

double calculate_point_B_y_in_triangle(double alpha, double beta, double offset_y, double a_length, double b_length) // berechnet den y-Wert aus zweidimensionaler Sicht (ohne x-Achse)
{
  double alpha_rad = alpha * PI / 180;
  double cos_alpha = cos(alpha_rad);

  double beta_rad = beta * PI / 180;
  double cos_beta = cos(beta_rad);

  double b1 = cos_alpha * b_length;
  double b2 = cos_beta * a_length;

  return b1 + b2 + offset_y; // B_y -> y-Wert von Punkt B
}

double calculate_point_B_z_in_triangle(double alpha, double beta, double offset_z, double a_length, double b_length) // berechnet den y-Wert aus zweidimensionaler Sicht (ohne z-Achse)
{
  double alpha_rad = alpha * PI / 180;
  double sin_alpha = sin(alpha_rad);

  double beta_rad = beta * PI / 180;
  double sin_beta = sin(beta_rad);

  double a1 = sin_alpha * b_length;
  double a2 = sin_beta * a_length;

  return a1 - a2 + offset_z; // B_z -> z-Wert von Punkt B
}

double calculate_angle_alpha_in_triangle_with_point(double position_y, double position_z, double offset_y, double offset_z, double a, double b)
{
  // double offset_y = 17.0;     // y-Koordinate Punkt A (Dreieck der Hebel)
  // double offset_z = 77.0;     // z-Koordinate Punkt A (Dreieck der Hebel)
  // double a = 100.0;           // Hebellänge a (Dreieck der Hebel)
  // double b = 100.0;           // Hebellänge b (Dreieck der Hebel)
  double a1 = a;
  double b1 = b;
  double a2 = abs(position_y - offset_y);
  double b2 = abs(position_z - offset_z);

  double c2 = sqrt(pow(a2, 2) + pow(b2, 2));
  double c1 = c2;

  double alpha2 = asin(a2 / c2) * 180 / PI;

  double beta2 = 180 - 90 - alpha2;

  double alpha1 = acos((pow(a1, 2) - pow(b1, 2) - pow(c1, 2)) / (-2 * a1 * c1)) * 180 / PI;

  double alpha_target;
  if (position_z < offset_z)
    alpha_target = alpha1 - beta2;
  else
    alpha_target = beta2 + alpha1;

  return alpha_target;
}

double calculate_angle_beta_in_triangle_with_point(double position_y, double position_z, double offset_y, double offset_z, double a, double b)
{
  // double offset_y = 17.0 ;   // y-Koordinate Punkt A (Dreieck der Hebel)
  // double offset_z = 77.0;   // z-Koordinate Punkt A (Dreieck der Hebel)
  // double a = 100.0;           // Hebellänge a (Dreieck der Hebel)
  // double b = 100.0;           // Hebellänge b (Dreieck der Hebel)
  double a1 = a;
  double b1 = b;
  double a2 = abs(position_y - offset_y);
  double b2 = abs(position_z - offset_z);

  double c2 = sqrt(pow(a2, 2) + pow(b2, 2));
  double c1 = c2;

  double alpha2 = asin(a2 / c2) * 180 / PI;

  double beta2 = 180 - 90 - alpha2;

  double alpha1 = acos((pow(a1, 2) - pow(b1, 2) - pow(c1, 2)) / (-2 * a1 * c1)) * 180 / PI;

  double gamma1 = acos((pow(c1, 2) - pow(a1, 2) - pow(b1, 2)) / (-2 * a1 * b1)) * 180 / PI;

  double alpha_target;
  if (position_z < offset_z)
    alpha_target = alpha1 - beta2;
  else
    alpha_target = beta2 + alpha1;

  double beta_target = 180 - alpha_target - gamma1;

  return beta_target;
}

double calculate_angle_alpha_in_triangle_with_distance(double distance_from_offset_to_target, double offset_z, double new_z, double a, double b)
{
  if (new_z - offset_z > 0)
  {
    double b_1 = new_z - offset_z;
    double beta_1 = asin(b_1 / distance_from_offset_to_target) * 180 / PI;
    double aplpha = acos((pow(a, 2) - pow(b, 2) - pow(distance_from_offset_to_target, 2)) / ((-2) * b * distance_from_offset_to_target)) * 180 / PI;
    double aplha_target = aplpha + beta_1;
    return aplha_target;
  }
  else
  {
    double a_1 = offset_z - new_z;
    double beta_1 = acos(a_1 / distance_from_offset_to_target) * 180 / PI;
    double angle_x = 90.0 - beta_1;
    double aplpha = acos((pow(a, 2) - pow(b, 2) - pow(distance_from_offset_to_target, 2)) / ((-2) * b * distance_from_offset_to_target)) * 180 / PI;
    double aplha_target = aplpha - angle_x;
    return aplha_target;
  }
}

double calculate_angle_beta_in_triangle_with_distance(double distance_from_offset_to_target, double offset_z, double new_z, double a, double b)
{
  if (new_z - offset_z > 0)
  {
    double b_1 = new_z - offset_z;
    double beta_1 = asin(b_1 / distance_from_offset_to_target) * 180 / PI;
    double alpha_1 = 180.0 - 90.0 - beta_1;
    double angle_x = 90.0 - alpha_1;
    double beta = acos((pow(b, 2) - pow(a, 2) - pow(distance_from_offset_to_target, 2)) / ((-2) * a * distance_from_offset_to_target)) * 180 / PI;
    double beta_target = beta - angle_x;
    return beta_target;
  }
  else
  {
    double a_1 = offset_z - new_z;
    double alpha_1 = asin(a_1 / distance_from_offset_to_target) * 180 / PI;
    double beta = acos((pow(b, 2) - pow(a, 2) - pow(distance_from_offset_to_target, 2)) / ((-2) * a * distance_from_offset_to_target)) * 180 / PI;
    double beta_target = beta + alpha_1;
    return beta_target;
  }
}

/************************************************************************************************************************
 *  Nachfolgendes bezieht sich auf den 3D-Raum
 ************************************************************************************************************************
 
                                                                         von oben auf Greifer blickend:              negative x-Achse 
    positive z-Achse                                                                                                       | -90°
           |                                                                                                               | 
           |                                                                                                          -----0----- 0°  y-Achse
           |                                                                                                               | 
           |                                                                                                               | +90°
           |                                                                                                         positive x-Achse 
           |                                                    
           |                                                                                           Drehung auf der z-Achse wird mit dem Winkel phi angegeben. Die positive y-Achse entspricht 0°.
           0------------------- positive y-Achse
          /  
         /    
        /
       / 
      /
  positive x-Achse 

  Nach dem Homing muss der Greifer in eine Grundstellung gefahren werden, wobei die jeweiligen Schritte der Motoren experimentell ermittelt werden müssen.
  Der Greifer muss in Grundstellung gerade nach vorn zeigen, exakt so wie die y-Achse verläuft.
  Diese Stellung ist 0° für Drehung auf der z-Achse.
  In Grundstellung werden die Winkel der Greiferarme händisch gemessen und im Programm hinterlegt.
  Dadurch können y- und z-Wert im Dreieck berechnet und diese, da noch in Grundstellung (x = 0), auch für den 3D-Raum übernommen werden.
  Soll ein x-Wert angefahren werden, muss der Drehwinkel auf der z-Achse berechnet und die neuen Punkte mittels Drehmatritze berechnet werden.

 
 
 */

double calculate_angle_phi(double target_x, double target_y)
{
  if (target_y != 0)
  {
    return atan(target_x / target_y) * 180 / PI;
  }
  else if (target_x < 0)
  {
    return -90.0;
  }
  else if (target_x > 0)
  {
    return 90.0;
  }
  else
    return 0.0;
}

// Offset-Vektor mit Drehmatritze um den Winkel Phi drehen

double calculate_x_part_of_new_offset_vector(double offset_vector_old_x, double offset_vector_old_y, double phi)
{
  double phi_rad = phi * PI / 180; // Umrechnung in RAD
  double result = cos(phi_rad) * offset_vector_old_x + (-1.0) * sin(phi_rad) * offset_vector_old_y;
  return result * (-1); // muss invertiert werden sonst stimmt das Vorzeichen der x-Xoordinate nicht mehr
}

double calculate_y_part_of_new_offset_vector(double offset_vector_old_x, double offset_vector_old_y, double phi)
{
  double phi_rad = phi * PI / 180; // Umrechnung in RAD
  double result = sin(phi_rad) * offset_vector_old_x + cos(phi_rad) * offset_vector_old_y;
  return result;
}

// Länge der Strecke von Spitze Offset-Vektor bis Zielpunkt berechnen

double calculate_distance_from_offset_to_target(double offset_x, double offset_y, double offset_z, double target_x, double target_y, double target_z)
{
  double x = offset_x - target_x;
  double y = offset_y - target_y;
  double z = offset_z - target_z;
  double result = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  return result;
}

void move_xM(double alpha_target, double *alpha_actual, const int x_dir, const int x_step, int gear_space_offset_steps) // Motor-x bewegen (Bezeichnung hat nichts mit x-Achse zu tun)
{
  const int STEP_DELAY = 2000;
  const int STEP_DELAY_GEAR_SPACE_OFFSET = 2000;
  const double STEPS_PER_REVOLUTION = 4096.0;
  const double DEGREE360 = 360.0;
  const int ONE_STEP = 1;
  static bool direction = 0; // 0 weil bei Kalibrierung dir = 0 ist

  if (*alpha_actual < alpha_target)
  {
    if (direction == 0)
    {
      move_motor(x_dir, x_step, 1, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("X-Offset");
    }
    static int x_1 = 0;
    direction = 1; // weil dir = 1 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*alpha_actual < alpha_target && *alpha_actual + DEGREE360 / STEPS_PER_REVOLUTION < alpha_target)
    {
      x_1++;
      Serial.print("x_1: ");
      Serial.println(x_1);
      move_motor(x_dir, x_step, 1, ONE_STEP, STEP_DELAY);
      *alpha_actual += DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
  else if (*alpha_actual > alpha_target)
  {
    if (direction == 1)
    {
      move_motor(x_dir, x_step, 0, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("X-Offset");
    }
    static int x_0 = 0;
    direction = 0; // weil dir = 0 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*alpha_actual > alpha_target && *alpha_actual - DEGREE360 / STEPS_PER_REVOLUTION > alpha_target)
    {
      x_0++;
      Serial.print("x_0: ");
      Serial.println(x_0);
      move_motor(x_dir, x_step, 0, ONE_STEP, STEP_DELAY);
      *alpha_actual -= DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
}

void move_yM(double beta_target, double *beta_actual, const int y_dir, const int y_step, int gear_space_offset_steps) // Motor-y bewegen (Bezeichnung hat nichts mit y-Achse zu tun)
{
  const int STEP_DELAY = 2000;
  const int STEP_DELAY_GEAR_SPACE_OFFSET = 2000;
  const double STEPS_PER_REVOLUTION = 4096.0;
  const double DEGREE360 = 360.0;
  const int ONE_STEP = 1;
  static bool direction = 1; // 1 weil bei Kalibrierung dir = 1 ist

  if (*beta_actual < beta_target)
  {
    if (direction == 1)
    {
      move_motor(y_dir, y_step, 0, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("Y-Offset");
    }
    static int y_0 = 0;
    direction = 0; // weil dir = 0 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*beta_actual < beta_target && *beta_actual + DEGREE360 / STEPS_PER_REVOLUTION < beta_target)
    {
      y_0++;
      Serial.print("y_0: ");
      Serial.println(y_0);
      move_motor(y_dir, y_step, 0, ONE_STEP, STEP_DELAY);
      *beta_actual += DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
  else if (*beta_actual > beta_target)
  {
    if (direction == 0)
    {
      move_motor(y_dir, y_step, 1, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("Y-Offset");
    }
    static int y_1 = 0;
    direction = 1; // weil dir = 1 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*beta_actual > beta_target && *beta_actual - DEGREE360 / STEPS_PER_REVOLUTION > beta_target)
    {
      y_1++;
      Serial.print("y_1: ");
      Serial.println(y_1);
      move_motor(y_dir, y_step, 1, ONE_STEP, STEP_DELAY);
      *beta_actual -= DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
}

void move_zM(double phi_target, double *phi_actual, const int z_dir, const int z_step, int gear_space_offset_steps) // Motor-z (Bezeichnung hat nichts mit z-Achse zu tun)
{
  const int STEP_DELAY = 1000;
  const int STEP_DELAY_GEAR_SPACE_OFFSET = 1000;
  const double STEPS_PER_REVOLUTION = 8192.0;
  const double DEGREE360 = 360.0;
  const int ONE_STEP = 1;
  static bool direction = 1; // 1 weil bei Kalibrierung dir = 1 ist

  if (*phi_actual < phi_target)
  {
    if (direction == 1)
    {
      move_motor(z_dir, z_step, 0, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("Z-Offset");
    }
    static int z_0 = 0;
    direction = 0; // weil dir = 0 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*phi_actual < phi_target && *phi_actual + DEGREE360 / STEPS_PER_REVOLUTION < phi_target)
    {
      z_0++;
      // Serial.print("z_0: ");
      // Serial.println(z_0);
      move_motor(z_dir, z_step, 0, ONE_STEP, STEP_DELAY);
      *phi_actual += DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
  else if (*phi_actual > phi_target)
  {
    if (direction == 0)
    {
      move_motor(z_dir, z_step, 1, gear_space_offset_steps, STEP_DELAY_GEAR_SPACE_OFFSET); // wenn vorher in die andere Richtung gefahren wurde, wird zuerst das Getriebespiel überwunden
      // Serial.println("Z-Offset");
    }
    static int z_1 = 0;
    direction = 1; // weil dir = 1 ist, Flag zeigt Richtungswechsel an, nötig für Ausgleich von Getriebespiel
    while (*phi_actual > phi_target && *phi_actual - DEGREE360 / STEPS_PER_REVOLUTION > phi_target)
    {
      z_1++;
      // Serial.print("z_1: ");
      // Serial.println(z_1);
      move_motor(z_dir, z_step, 1, ONE_STEP, STEP_DELAY);
      *phi_actual -= DEGREE360 / STEPS_PER_REVOLUTION;
    }
  }
}

