// QuadPIDController.cpp
#include "Utility/QuadPIDController.h"
#include "Math/UnrealMathUtility.h"

QuadPIDController::QuadPIDController()
    : ProportionalGain(0.0f)
    , IntegralGain(0.0f)
    , DerivativeGain(0.0f)
    , prevError(0.0f)
    , lastOutput(0.0f)
    , absoluteTime(0.0f)
    , currentBufferSum(0.0f)
    , minOutput(0.0f)
    , maxOutput(1.0f)
    , filteredDerivative(0.0f)
    , derivativeFilterAlpha(0.6f)
{
    // Pre-allocate buffer to avoid reallocations
    integralBuffer.Reserve(ESTIMATED_BUFFER_SIZE);
}

void QuadPIDController::SetGains(float pGain, float iGain, float dGain, float filterAlpha)
{
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
    
    SetDerivativeFilterAlpha(filterAlpha);
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

    // Calculate raw derivative
    double rawDerivative = (error - prevError) / dt;
    
    // Apply low-pass filter to derivative term
    filteredDerivative = derivativeFilterAlpha * rawDerivative + (1.0f - derivativeFilterAlpha) * filteredDerivative;
    
    // Derivative term using filtered derivative
    double d_term = DerivativeGain * filteredDerivative;

    // Combine terms and clamp output
    double output = p_term + i_term + d_term;
    output = FMath::Clamp(output, minOutput, maxOutput);
    
    prevError = error;
    lastOutput = output;

    // Debug logging
    UE_LOG(LogTemp, VeryVerbose, TEXT("Time: %.3f, Buffer Size: %d, Sum: %.4f, Raw D: %.4f, Filtered D: %.4f"), 
           absoluteTime,
           integralBuffer.Num(), 
           currentBufferSum,
           rawDerivative,
           filteredDerivative);

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
    filteredDerivative = 0.0f;  // Reset the filtered derivative
}

void QuadPIDController::ResetIntegral()
{
    integralBuffer.Empty();
    currentBufferSum = 0.0f;
}

// New method to set the derivative filter coefficient
void QuadPIDController::SetDerivativeFilterAlpha(float alpha)
{
    // Keep alpha in valid range [0.0, 1.0]
    derivativeFilterAlpha = FMath::Clamp(alpha, 0.0f, 1.0f);
}