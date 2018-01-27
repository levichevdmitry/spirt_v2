
#ifndef  _PID_H_
#define  _PID_H_

typedef struct
{
  double dState;                  // Last position input
  double iState;                  // Integrator state
  double iMax, iMin;
  // Maximum and minimum allowable integrator state
  double    iGain,        // integral gain
            pGain,        // proportional gain
            dGain;         // derivative gain
	double outMin, outMax;
} SPid;

double UpdatePID(SPid * pid, double error, double position);

void SetKpPID(SPid * pid, double Kp);
void SetKiPID(SPid * pid, double Ki);
void SetKdPID(SPid * pid, double Kd);
void SetOutLimPID(SPid * pid, double outMin, double outMax);
void ResetiStatePID(SPid * pid);
#endif
