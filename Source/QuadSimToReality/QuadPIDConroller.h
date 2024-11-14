#pragma once

#include "CoreMinimal.h"

class QUADSIMTOREALITY_API QuadPIDController
{
public:
	QuadPIDController();

	void SetGains(float pGain, float iGain, float dGain);
	void SetLimits(float min_output, float max_output);
	void Reset();
	void ResetIntegral();

	float Calculate(float error, float dt);

	// PID Gains
	float ProportionalGain;
	float IntegralGain;
	float DerivativeGain;
	
	// Last output for debugging or logging
	float lastOutput;

private:
	float minOutput;
	float maxOutput;
	float integralSum;
	float prevError;
};