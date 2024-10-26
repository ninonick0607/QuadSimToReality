// QuadPIDController.cpp

// #include "QuadPIDConroller.h"
// #include "Math/UnrealMathUtility.h"
// DEFINE_LOG_CATEGORY(PositionPIDLog);
// DEFINE_LOG_CATEGORY(AttitudePIDLog);

#include "QuadPIDConroller.h"
#include "Math/UnrealMathUtility.h"

QuadPIDController::QuadPIDController()
    : ProportionalGain(0.0f), IntegralGain(0.0f), DerivativeGain(0.0f),
      minOutput(-1.0f), maxOutput(1.0f),
      integralSum(0.0f), prevError(0.0f), lastOutput(0.0f)
{
}

void QuadPIDController::SetGains(float pGain, float iGain, float dGain)
{
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
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
// QuadPIDController::QuadPIDController()
//     : attitudeControl(false), minOutput(-1.0f), maxOutput(1.0f),
//       integralSum(0.0f), prevError(0.0f)
// {
// }
//
// void QuadPIDController::SetGains(float pGain, float iGain, float dGain)
// {
//     if (attitudeControl)
//     {
//         attitudeProportionalGain = pGain;
//         attitudeIntegralGain = iGain;
//         attitudeDerivativeGain = dGain;
//     }
//     else
//     {
//         positionProportionalGain = pGain;
//         positionIntegralGain = iGain;
//         positionDerivativeGain = dGain;
//     }
// }
//
// float QuadPIDController::Calculate(float error, float dt)
// {
//     if (dt <= KINDA_SMALL_NUMBER)
//     {
//         UE_LOG(LogTemp, Warning, TEXT("QuadPIDController::Calculate called with dt <= KINDA_SMALL_NUMBER"));
//         return 0.0f;
//     }
//
//     // Determine which gains to use based on the control type
//     float pGain = attitudeControl ? attitudeProportionalGain : positionProportionalGain;
//     float iGain = attitudeControl ? attitudeIntegralGain : positionIntegralGain;
//     float dGain = attitudeControl ? attitudeDerivativeGain : positionDerivativeGain;
//
//     // Calculate Proportional term
//     float p_term = pGain * error;
//
//     // Calculate Integral term with clamping to prevent windup
//     integralSum += error * dt;
//     float maxIntegral = 100.0f; // Adjust based on system requirements
//     integralSum = FMath::Clamp(integralSum, -maxIntegral, maxIntegral);
//     float i_term = iGain * integralSum;
//
//     // Calculate Derivative term
//     float d_term = dGain * (error - prevError) / dt;
//
//     // Calculate the raw output before clamping
//     float output = p_term + i_term + d_term;
//
//     // Clamp output
//     output = FMath::Clamp(output, minOutput, maxOutput);
//
//     // Update previous error
//     prevError = error;
//
//     // Store the last output
//     lastOutput = output;
//
//     // **Conditional Logging**
//     if (!attitudeControl)
//     {
//         UE_LOG(PositionPIDLog, Log, TEXT("Position PID Controller Outputs:"));
//         UE_LOG(PositionPIDLog, Log, TEXT("  P Term: %.2f"), p_term);
//         UE_LOG(PositionPIDLog, Log, TEXT("  I Term: %.2f"), i_term);
//         UE_LOG(PositionPIDLog, Log, TEXT("  D Term: %.2f"), d_term);
//         UE_LOG(PositionPIDLog, Log, TEXT("  Total Output: %.2f"), output);
//     }
//     else
//     {
//         UE_LOG(AttitudePIDLog, Log, TEXT("Attitude PID Controller Outputs:"));
//         UE_LOG(AttitudePIDLog, Log, TEXT("  P Term: %.2f"), p_term);
//         UE_LOG(AttitudePIDLog, Log, TEXT("  I Term: %.2f"), i_term);
//         UE_LOG(AttitudePIDLog, Log, TEXT("  D Term: %.2f"), d_term);
//         UE_LOG(AttitudePIDLog, Log, TEXT("  Total Output: %.2f"), output);
//     }
//
//     return output;
// }

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


