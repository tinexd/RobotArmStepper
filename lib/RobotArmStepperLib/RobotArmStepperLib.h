#ifndef RobotArmStepperLib_h
#define RobotArmStepperLib_h

#include <Arduino.h>

void move_motor(int dir_pin, int step_pin, bool dir, int steps, int delay);

void homing(const int x_step, const int x_dir, const int y_step, const int y_dir, const int z_step, const int z_dir, const int enPin,
            const int x_homePin, const int y_homePin, const int z_homePin);

void move_xM(double alpha_target, double *alpha_actual, const int x_dir, const int x_step, int gear_space_offset_steps);

void move_yM(double beta_target, double *beta_actual, const int y_dir, const int y_step, int gear_space_offset_steps);

void move_zM(double phi_target, double *phi_actual, const int z_dir, const int z_step, int gear_space_offset_steps);

double calculate_angle_alpha_in_triangle_with_point(double position_y, double position_z, double offset_y, double offset_z, double a, double b);

double calculate_angle_beta_in_triangle_with_point(double position_y, double position_z, double offset_y, double offset_z, double a, double b);

double calculate_angle_alpha_in_triangle_with_distance(double distance_from_offset_to_target, double offset_z, double new_z, double a, double b);

double calculate_angle_beta_in_triangle_with_distance(double distance_from_offset_to_target, double offset_z, double new_z, double a, double b);

double calculate_angle_phi(double target_x, double target_y);

double calculate_point_B_y_in_triangle(double alpha, double beta, double offset_y, double a_length, double b_length);

double calculate_point_B_z_in_triangle(double alpha, double beta, double A_z, double a_length, double b_length);

double calculate_x_part_of_new_offset_vector(double offset_vector_old_x, double offset_vector_old_y, double phi);

double calculate_y_part_of_new_offset_vector(double offset_vector_old_x, double offset_vector_old_y, double phi);

double calculate_distance_from_offset_to_target(double offset_x, double offset_y, double offset_z, double target_x, double target_y, double target_z);

void move_straight_to_point(double *ptr_alpha_actual, double *ptr_beta_actual, double start_point_x, double start_point_y, double target_position_x, double target_position_y,
                            const int x_step, const int x_dir, const int y_step, const int y_dir, const int z_step, const int z_dir,
                            double A_y, double a_length, double b_length);



#endif