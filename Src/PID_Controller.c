float Ki_temp = 0, pre_error = 0, interal;

float PID_Controller(float set_value, float mea_value, float max_value, float min_value, float Kp, float Ki, float Kd, float Ts)
{
	float output, error;
	float derivative;
	
	error = set_value - mea_value;
	interal = interal + error*Ts;
	derivative = (error - pre_error)/Ts;
	output = Kp*error + Ki*interal + Kd*derivative;
	
	if (output > max_value)
	{
		output = max_value;
	} 
	else if ( output < min_value)
	{
		output = min_value;
	}
	
	pre_error = error;
	
	return output;
}
