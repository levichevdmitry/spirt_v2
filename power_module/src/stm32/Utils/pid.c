
#include <pid.h>

double UpdatePID(SPid * pid, double error, double position)
{
  double pTerm, dTerm, iTerm, pidOut;
 
  pTerm = pid->pGain * error;    // calculate the proportional term
  pid->iState += error;          // calculate the integral state with appropriate limiting
 
  if (pid->iState > pid->iMax) 
      pid->iState = pid->iMax;     
  else if (pid->iState < pid->iMin) 
      pid->iState = pid->iMin;
  iTerm = pid->iGain * pid->iState;    // calculate the integral term
  dTerm = pid->dGain * (position - pid->dState);
  pid->dState = position;
	pidOut = (pTerm + iTerm - dTerm);
	// limit output
	if (pidOut < pid->outMin) {
		pidOut = pid->outMin;
	}
	if (pidOut > pid->outMax){
		pidOut = pid->outMax;
	}
  return pidOut;
}

void SetKpPID(SPid * pid, double Kp){
	pid->pGain = Kp;
}

void SetKiPID(SPid * pid, double Ki){
	pid->iGain = Ki;
	pid->iMax = pid->outMax / Ki;
	pid->iMin = pid->outMin / Ki;
}

void SetKdPID(SPid * pid, double Kd){
	pid->dGain = Kd;
}

void SetOutLimPID(SPid * pid, double outMin, double outMax){
	pid->outMax = outMax;
	pid->outMin = outMin;
}

void ResetiStatePID(SPid * pid){
	pid->iState = 0.0;
}

