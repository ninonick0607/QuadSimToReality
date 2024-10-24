// QuadPIDController.h

#pragma once

#include "CoreMinimal.h"
 
/**
 * A simple PID Controller class used for controlling drone movements.
 */

DECLARE_LOG_CATEGORY_EXTERN(PositionPIDLog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(AttitudePIDLog, Log, All);

class QUADSIMTOREALITY_API QuadPIDController
{
public:
    QuadPIDController();

    void SetGains(float pGain, float iGain, float dGain);
    void SetLimits(float min_output, float max_output);
    void Reset();

    float Calculate(float error, float dt);
	// Static gain variables
	float positionProportionalGain = 2.5f;
	float positionIntegralGain = 0.3f;
	float positionDerivativeGain = 0.16f;

	float attitudeProportionalGain = 0.1f;
	float attitudeIntegralGain = 0.19f;
	float attitudeDerivativeGain = 4.1320f;

    bool attitudeControl;
	float lastOutput;

private:
    float minOutput;
    float maxOutput;
    float integralSum;
    float prevError;
	
};
