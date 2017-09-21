#include <stdlib.h>
#include <math.h>

#define PI 3.1416f
float pitch_qs, roll_qs, yaw_qs;
extern float magX_bias, magY_bias;

void imu9dof(float* acc_in, float* gyro_in, float* mag_in, float dt, float* out)
{
	float Q_angle = 0.001f, Q_bias = 0.003f, R_measure = 0.001f;
	float Q_angle_yaw = 0.01f, Q_bias_yaw = 0.03f, R_measure_yaw = 0.001f;
	static float pitch = 0.0f, roll = 0.0f, yaw = 0.0f, bias_pitch = 0.0f, bias_roll = 0.0f, bias_yaw = 0.0f;
	static float P_pitch[2][2], P_roll[2][2], P_yaw[2][2];
	
	// Pitch
	P_pitch[0][0] = 0.0f;
  P_pitch[0][1] = 0.0f;
  P_pitch[1][0] = 0.0f;
  P_pitch[1][1] = 0.0f;
	
	gyro_in[1] = -gyro_in[1] - bias_pitch;
	pitch += dt*gyro_in[1];
	
	P_pitch[0][0] += dt * (dt*P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
	P_pitch[0][1] -= dt * P_pitch[1][1];
	P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_bias * dt;
	
	pitch_qs = -atan2(-acc_in[1], sqrt((acc_in[0]*acc_in[0])+(acc_in[2]*acc_in[2])))*180/PI;
	float y_pitch = pitch_qs - pitch;
	
	float S_pitch = P_pitch[0][0] + R_measure;
	
	float K_pitch[2];
	K_pitch[0] = P_pitch[0][0] / S_pitch;
  K_pitch[1] = P_pitch[1][0] / S_pitch;
	
	pitch += K_pitch[0] * y_pitch;
  bias_pitch += K_pitch[1] * y_pitch;
	
	float P00_temp_pitch = P_pitch[0][0];
  float P01_temp_pitch = P_pitch[0][1];

  P_pitch[0][0] -= K_pitch[0] * P00_temp_pitch;
  P_pitch[0][1] -= K_pitch[0] * P01_temp_pitch;
  P_pitch[1][0] -= K_pitch[1] * P00_temp_pitch;
  P_pitch[1][1] -= K_pitch[1] * P01_temp_pitch;
	
	//Roll
	P_roll[0][0] = 0.0f;
  P_roll[0][1] = 0.0f;
  P_roll[1][0] = 0.0f;
  P_roll[1][1] = 0.0f;
	
	gyro_in[0] = gyro_in[0] - bias_roll;
	roll += dt*gyro_in[0];
	
	P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
	P_roll[0][1] -= dt * P_roll[1][1];
	P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_bias * dt;
	
	roll_qs = atan2(-acc_in[0], sqrt((acc_in[1]*acc_in[1])+(acc_in[2]*acc_in[2])))*180/PI;
	float y_roll = roll_qs - roll;
	
	float S_roll = P_roll[0][0] + R_measure;
	
	float K_roll[2];
	K_roll[0] = P_roll[0][0] / S_roll;
  K_roll[1] = P_roll[1][0] / S_roll;
	
	roll += K_roll[0] * y_roll;
  bias_roll += K_roll[1] * y_roll;
	
	float P00_temp_roll = P_roll[0][0];
  float P01_temp_roll = P_roll[0][1];

  P_roll[0][0] -= K_roll[0] * P00_temp_roll;
  P_roll[0][1] -= K_roll[0] * P01_temp_roll;
  P_roll[1][0] -= K_roll[1] * P00_temp_roll;
  P_roll[1][1] -= K_roll[1] * P01_temp_roll;
	
	//Yaw
	
	P_yaw[0][0] = 0.0f;
  P_yaw[0][1] = 0.0f;
  P_yaw[1][0] = 0.0f;
  P_yaw[1][1] = 0.0f;
	
	gyro_in[2] = -gyro_in[2] - bias_yaw;
	yaw += dt*gyro_in[2];
	
	P_yaw[0][0] += dt * (dt*P_yaw[1][1] - P_yaw[0][1] - P_yaw[1][0] + Q_angle_yaw);
	P_yaw[0][1] -= dt * P_yaw[1][1];
	P_yaw[1][0] -= dt * P_yaw[1][1];
  P_yaw[1][1] += Q_bias_yaw * dt;
	
	yaw_qs = -atan2(mag_in[0] - magX_bias,mag_in[1] - magY_bias)*180/PI;
	if (yaw_qs < 0)
	{
		yaw_qs += 360;
	} else if (yaw_qs > 360)
	{
		yaw_qs -= 360;
	}
	float y = yaw_qs - yaw;
	
	float S_yaw = P_yaw[0][0] + R_measure_yaw;
	
	float K_yaw[2];
	K_yaw[0] = P_yaw[0][0] / S_yaw;
  K_yaw[1] = P_yaw[1][0] / S_yaw;
	
	yaw += K_yaw[0] * y;
  bias_yaw += K_yaw[1] * y;
	
	float P00_temp_yaw = P_yaw[0][0];
  float P01_temp_yaw = P_yaw[0][1];

  P_yaw[0][0] -= K_yaw[0] * P00_temp_yaw;
  P_yaw[0][1] -= K_yaw[0] * P01_temp_yaw;
  P_yaw[1][0] -= K_yaw[1] * P00_temp_yaw;
  P_yaw[1][1] -= K_yaw[1] * P01_temp_yaw;
	
	out[0] = pitch;
	out[1] = roll;
	out[2] = yaw;
}
