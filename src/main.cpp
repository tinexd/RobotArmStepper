// Stepper-Driver in 1/2-Mode

#include <Arduino.h>
#include <RobotArmStepperLib.h>

const int x_step = 2;
const int x_dir = 5;
const int y_step = 3;
const int y_dir = 6;
const int z_step = 4;
const int z_dir = 7;
const int enPin = 8;
const int x_homePin = 9;
const int y_homePin = 10;
const int z_homePin = 11;

#define X_GEAR_SPACE_OFFSET_STEPS 35
#define Y_GEAR_SPACE_OFFSET_STEPS 35
#define Z_GEAR_SPACE_OFFSET_STEPS 0

double alpha_actual;
double beta_actual;
double phi_actual = 0.0;

double a = 100.0; // in mm
double b = 100.0;

double offset_z = 77.0; // in mm
double offset_y = 17.0;

double offset_vector_x = 0.0; // da Greifer nach Neustart in y-Achse ausgerichtet ist -> x=0
double offset_vector_y = offset_y;

double start_point_x;
double start_point_y;
double start_point_z;

double target_position_x;
double target_position_y;
double target_position_z;

void move_straight_to_point(double start_point_x, double start_point_y, double start_point_z, double target_position_x, double target_position_y, double target_position_z);

void setup()
{
  pinMode(x_step, OUTPUT);
  pinMode(x_dir, OUTPUT);
  pinMode(y_step, OUTPUT);
  pinMode(y_dir, OUTPUT);
  pinMode(z_step, OUTPUT);
  pinMode(z_dir, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(x_homePin, INPUT_PULLUP);
  pinMode(y_homePin, INPUT_PULLUP);
  pinMode(z_homePin, INPUT_PULLUP);

  digitalWrite(enPin, LOW); // Schrittmotoren-Treiber einschalten

  Serial.begin(9600);

  homing(x_step, x_dir, y_step, y_dir, z_step, z_dir, enPin, x_homePin, y_homePin, z_homePin);

  // Kalibrierung *********************************
  move_motor(z_dir, z_step, 1, 2080, 1000); // Werte durch Erproben ermitteln, der Greifer muss in Richtung der y-Achse ausgreichtet sein
  move_motor(x_dir, x_step, 0, 360, 2000);  // Werte durch Erproben ermitteln, die Greiferarme müssen im unten eingegebenen Winkel stehen
  move_motor(y_dir, y_step, 1, 300, 2000);  // Werte durch Erproben ermitteln, die Greiferarme müssen im unten eingegebenen Winkel stehen
  alpha_actual = 45.0;                      // hier Winkel eintragen
  beta_actual = 45.0;                       // hier Winkel eintragen
  // **********************************************

  start_point_x = 0.0;
  start_point_y = calculate_point_B_y_in_triangle(alpha_actual, beta_actual, offset_y, a, b);
  start_point_z = calculate_point_B_z_in_triangle(alpha_actual, beta_actual, offset_z, a, b);
  Serial.println();
  Serial.print("start_point_x: ");
  Serial.println(start_point_x);
  Serial.print("start_point_y: ");
  Serial.println(start_point_y);
  Serial.print("start_point_z: ");
  Serial.println(start_point_z);
  Serial.println();
  delay(5000);

  // ***************************************************************************

  target_position_x = 0.0;
  target_position_y = 158.42;
  target_position_z = 20.0;

  move_straight_to_point(start_point_x, start_point_y, start_point_z, target_position_x, target_position_y, target_position_z);

  start_point_x = target_position_x;
  start_point_y = target_position_y;
  start_point_z = target_position_z;
}

void loop()
{
}

void move_straight_to_point(double start_point_x, double start_point_y, double start_point_z, double target_position_x, double target_position_y, double target_position_z)
{
  double waypoint_x = start_point_x;
  double waypoint_y = start_point_y;
  double waypoint_z = start_point_z;

  double support_vector_x_value = start_point_x; // x-Wert Stützvektor
  double support_vector_y_value = start_point_y; // y-Wert Stützvektor
  double support_vector_z_value = start_point_z; // z-Wert Stützvektor

  double direction_vector_x_value = target_position_x - start_point_x; // x-Wert Richtungsvektor = Spize minus Fuß
  double direction_vector_y_value = target_position_y - start_point_y; // y-wert Richtungsvektor = Spize minus Fuß
  double direction_vector_z_value = target_position_z - start_point_z; // z-wert Richtungsvektor = Spize minus Fuß

  double length_straight_line = sqrt(pow(start_point_x - target_position_x, 2.0) + pow(start_point_y - target_position_y, 2.0) + pow(start_point_z - target_position_z, 2.0)); // Länge der Geraden mit Satz des Pythagoras

  double step_size = 0.1;
  double factor = 0;
  double distance_actually_travelled = 0.0;

  while (distance_actually_travelled + step_size < length_straight_line)
  {

    factor += step_size / length_straight_line;
    waypoint_x = support_vector_x_value + factor * direction_vector_x_value; // x-Wert von Punkt auf Geraden mit Geradengleichung ermittelt
    waypoint_y = support_vector_y_value + factor * direction_vector_y_value; // y-Wert von Punkt auf Geraden mit Geradengleichung ermittelt
    waypoint_z = support_vector_z_value + factor * direction_vector_z_value; // z-Wert von Punkt auf Geraden mit Geradengleichung ermittelt

    double phi_target = calculate_angle_phi(waypoint_x, waypoint_y);
    move_zM(phi_target, &phi_actual, z_dir, z_step, Z_GEAR_SPACE_OFFSET_STEPS);

    double x_part_of_new_offset_vector = calculate_x_part_of_new_offset_vector(offset_vector_x, offset_vector_y, phi_actual);
    double y_part_of_new_offset_vector = calculate_y_part_of_new_offset_vector(offset_vector_x, offset_vector_y, phi_actual);

    double distance_from_offset_to_target = calculate_distance_from_offset_to_target(x_part_of_new_offset_vector, y_part_of_new_offset_vector, offset_z, waypoint_x, waypoint_y, waypoint_z);

    double alpha_target = calculate_angle_alpha_in_triangle_with_distance(distance_from_offset_to_target, offset_z, waypoint_z, a, b);
    double beta_target = calculate_angle_beta_in_triangle_with_distance(distance_from_offset_to_target, offset_z, waypoint_z, a, b);

    move_xM(alpha_target, &alpha_actual, x_dir, x_step, X_GEAR_SPACE_OFFSET_STEPS);
    move_yM(beta_target, &beta_actual, y_dir, y_step, Y_GEAR_SPACE_OFFSET_STEPS);

    distance_actually_travelled = factor * length_straight_line;
  }

  if (target_position_x != waypoint_y || target_position_y != waypoint_y || target_position_z != waypoint_z)

  {
    waypoint_x = target_position_x;
    waypoint_y = target_position_y;
    waypoint_z = target_position_z;

    double phi_target = calculate_angle_phi(waypoint_x, waypoint_y);

    move_zM(phi_target, &phi_actual, z_dir, z_step, Z_GEAR_SPACE_OFFSET_STEPS);

    double x_part_of_new_offset_vector = calculate_x_part_of_new_offset_vector(offset_vector_x, offset_vector_y, phi_actual);
    double y_part_of_new_offset_vector = calculate_y_part_of_new_offset_vector(offset_vector_x, offset_vector_y, phi_actual);

    double distance_from_offset_to_target = calculate_distance_from_offset_to_target(x_part_of_new_offset_vector, y_part_of_new_offset_vector, offset_z, waypoint_x, waypoint_y, waypoint_z);

    double alpha_target = calculate_angle_alpha_in_triangle_with_distance(distance_from_offset_to_target, offset_z, waypoint_z, a, b);
    double beta_target = calculate_angle_beta_in_triangle_with_distance(distance_from_offset_to_target, offset_z, waypoint_z, a, b);

    move_xM(alpha_target, &alpha_actual, x_dir, x_step, X_GEAR_SPACE_OFFSET_STEPS);
    move_yM(beta_target, &beta_actual, y_dir, y_step, Y_GEAR_SPACE_OFFSET_STEPS);
  }
}