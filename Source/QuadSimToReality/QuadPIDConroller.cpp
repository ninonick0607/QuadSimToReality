// QuadPIDController.cpp
#include "QuadPIDConroller.h"
#include "Math/UnrealMathUtility.h"

QuadPIDController::QuadPIDController()
    : ProportionalGain(0.0f), IntegralGain(0.0f), DerivativeGain(0.0f),
      minOutput(0.0f), maxOutput(1.0f),
      integralSum(0.0f), prevError(0.0f), lastOutput(0.0f)
{
}

void QuadPIDController::SetGains(float pGain, float iGain, float dGain)
{
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
}
void QuadPIDController::ResetIntegral()
{
    integralSum = 0.0f;
}
float QuadPIDController::Calculate(float error, float dt)
{
    if (dt <= KINDA_SMALL_NUMBER)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPIDController::Calculate called with dt <= KINDA_SMALL_NUMBER"));
        return 0.0f;
    }

    // Proportional term
    float p_term = ProportionalGain * error;

    // Integral term
    integralSum += error * dt;
    // new waypoint zero out sum
    float maxIntegral = 100.0f;
    integralSum = FMath::Clamp(integralSum, -maxIntegral, maxIntegral);
    float i_term = IntegralGain * integralSum;

    // Derivative term
    float d_term = DerivativeGain * (error - prevError) / dt;

    // Total output
    float output = p_term + i_term + d_term;

    // Clamp output
    output = FMath::Clamp(output, minOutput, maxOutput);

    // Update previous error
    prevError = error;

    // Store last output
    lastOutput = output;

    return output;
}

void QuadPIDController::SetLimits(float min_output, float max_output)
{
    minOutput = min_output;
    maxOutput = max_output;
}

void QuadPIDController::Reset()
{
    integralSum = 0.0f;
    prevError = 0.0f;
}
