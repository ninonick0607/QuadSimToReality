
// QuadPIDController.cpp
#include "Utility/QuadPIDConroller.h"
#include "Math/UnrealMathUtility.h"

QuadPIDController::QuadPIDController()
    : ProportionalGain(0.0f)
    , IntegralGain(0.0f)
    , DerivativeGain(0.0f)
    , minOutput(0.0f)
    , maxOutput(1.0f)
    , prevError(0.0f)
    , lastOutput(0.0f)
    , absoluteTime(0.0f)
    , currentBufferSum(0.0f)
{
    // Pre-allocate buffer to avoid reallocations
    integralBuffer.Reserve(ESTIMATED_BUFFER_SIZE);
}

void QuadPIDController::SetGains(float pGain, float iGain, float dGain)
{
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
}

void QuadPIDController::RemoveExpiredPoints()
{
    const float windowStart = absoluteTime - INTEGRAL_WINDOW_DURATION;
    
    // Remove points older than the window duration
    while (!integralBuffer.IsEmpty() && integralBuffer[0].timestamp < windowStart)
    {
        currentBufferSum -= integralBuffer[0].value;
        integralBuffer.RemoveAt(0, 1, EAllowShrinking::No);
    }
}


double QuadPIDController::Calculate(float error, float dt)
{
    if (dt <= KINDA_SMALL_NUMBER)
    {
        UE_LOG(LogTemp, Warning, TEXT("QuadPIDController::Calculate called with dt <= KINDA_SMALL_NUMBER"));
        return 0.0f;
    }

    absoluteTime += dt;

    // Remove expired points from the sliding window
    RemoveExpiredPoints();

    // Proportional term
    double p_term = ProportionalGain * error;

    // Add new integral point
    IntegralPoint newPoint;
    newPoint.timestamp = absoluteTime;
    newPoint.value = error * dt;
    
    integralBuffer.Add(newPoint);
    currentBufferSum += newPoint.value;

    // Integral term with sliding window
    double i_term = IntegralGain * currentBufferSum;

    // Derivative term
    double d_term = DerivativeGain * (error - prevError) / dt;

    // Combine terms and clamp output
    double output = p_term + i_term + d_term;
    output = FMath::Clamp(output, minOutput, maxOutput);
    
    prevError = error;
    lastOutput = output;

    // Debug logging
    UE_LOG(LogTemp, VeryVerbose, TEXT("Time: %.3f, Buffer Size: %d, Sum: %.4f"), 
           absoluteTime,
           integralBuffer.Num(), 
           currentBufferSum);

    return output;
}

void QuadPIDController::SetLimits(float min_output, float max_output)
{
    minOutput = min_output;
    maxOutput = max_output;
}

void QuadPIDController::Reset()
{
    integralBuffer.Empty();
    currentBufferSum = 0.0f;
    prevError = 0.0f;
    absoluteTime = 0.0f;
}

void QuadPIDController::ResetIntegral()
{
    integralBuffer.Empty();
    currentBufferSum = 0.0f;
}