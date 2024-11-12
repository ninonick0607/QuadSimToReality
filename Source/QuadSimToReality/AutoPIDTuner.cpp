
// PIDAutoTuner.cpp
#include "AutoPIDTuner.h"

AutoPIDTuner::AutoPIDTuner()
    : tuningComplete(false)
    , currentIteration(0)
    , maxIterations(1000)
    , convergenceThreshold(0.01f)
    , initialDelta(0.1f)
    , parameterIndex(0)
{
    currentDeltaP = initialDelta;
    currentDeltaI = initialDelta;
    currentDeltaD = initialDelta;
}

void AutoPIDTuner::StartTuning(QuadPIDController* xPID, QuadPIDController* yPID, QuadPIDController* zPID,
    QuadPIDController* rollPID, QuadPIDController* pitchPID, QuadPIDController* yawPID)
{
    pidControllers[0] = xPID;
    pidControllers[1] = yPID;
    pidControllers[2] = zPID;
    pidControllers[3] = rollPID;
    pidControllers[4] = pitchPID;
    pidControllers[5] = yawPID;

    // Initialize with current parameters
    currentParameters.position.P = xPID->ProportionalGain;
    currentParameters.position.I = xPID->IntegralGain;
    currentParameters.position.D = xPID->DerivativeGain;

    currentParameters.attitude.P = rollPID->ProportionalGain;
    currentParameters.attitude.I = rollPID->IntegralGain;
    currentParameters.attitude.D = rollPID->DerivativeGain;

    bestParameters = currentParameters;
    tuningComplete = false;
    currentIteration = 0;
    performanceHistory.Empty();
}

void AutoPIDTuner::UpdateTuning(const FVector& currentPosition, const FVector& targetPosition,
    const FRotator& currentRotation, const FRotator& targetRotation,
    float deltaTime)
{
    if (tuningComplete) return;

    // Evaluate current performance
    float performance = EvaluatePerformance(currentPosition, targetPosition, currentRotation, targetRotation);
    performanceHistory.Add(performance);

    // Keep track of best parameters
    if (performance > bestParameters.fitness) {
        bestParameters = currentParameters;
        bestParameters.fitness = performance;

        // Increase delta for this parameter
        switch (parameterIndex % 3) {
        case 0: currentDeltaP *= 1.1f; break;
        case 1: currentDeltaI *= 1.1f; break;
        case 2: currentDeltaD *= 1.1f; break;
        }
    }
    else {
        // Decrease delta and try opposite direction
        switch (parameterIndex % 3) {
        case 0: currentDeltaP *= 0.9f; break;
        case 1: currentDeltaI *= 0.9f; break;
        case 2: currentDeltaD *= 0.9f; break;
        }
        UpdateParameter(parameterIndex, -currentDeltaP);
    }

    // Move to next parameter
    parameterIndex = (parameterIndex + 1) % 6;

    // Check for convergence or iteration limit
    currentIteration++;
    if (currentIteration >= maxIterations ||
        (currentDeltaP < convergenceThreshold &&
            currentDeltaI < convergenceThreshold &&
            currentDeltaD < convergenceThreshold)) {
        tuningComplete = true;
        ApplyParameters(bestParameters);
    }
}

float AutoPIDTuner::EvaluatePerformance(const FVector& currentPosition, const FVector& targetPosition,
    const FRotator& currentRotation, const FRotator& targetRotation)
{
    // Calculate position error
    FVector posError = targetPosition - currentPosition;
    float positionError = posError.Size();

    // Calculate rotation error
    FRotator rotError = targetRotation - currentRotation;
    float rotationError = rotError.Euler().Size();

    // Combine errors with weighting
    float positionWeight = 0.6f;
    float rotationWeight = 0.4f;

    // Convert to a fitness value (higher is better)
    float fitness = 1.0f / (1.0f + positionWeight * positionError + rotationWeight * rotationError);

    // Add stability penalty if necessary
    if (!CheckStability()) {
        fitness *= 0.5f;
    }

    return fitness;
}

void AutoPIDTuner::UpdateParameter(int index, float delta)
{
    bool isPosition = index < 3;
    int subIndex = index % 3;

    if (isPosition) {
        switch (subIndex) {
        case 0: currentParameters.position.P += delta; break;
        case 1: currentParameters.position.I += delta; break;
        case 2: currentParameters.position.D += delta; break;
        }
    }
    else {
        switch (subIndex) {
        case 0: currentParameters.attitude.P += delta; break;
        case 1: currentParameters.attitude.I += delta; break;
        case 2: currentParameters.attitude.D += delta; break;
        }
    }

    // Ensure parameters stay positive
    currentParameters.position.P = FMath::Max(0.0f, currentParameters.position.P);
    currentParameters.position.I = FMath::Max(0.0f, currentParameters.position.I);
    currentParameters.position.D = FMath::Max(0.0f, currentParameters.position.D);
    currentParameters.attitude.P = FMath::Max(0.0f, currentParameters.attitude.P);
    currentParameters.attitude.I = FMath::Max(0.0f, currentParameters.attitude.I);
    currentParameters.attitude.D = FMath::Max(0.0f, currentParameters.attitude.D);

    ApplyParameters(currentParameters);
}

void AutoPIDTuner::ApplyParameters(const TuningParameters& params)
{
    // Apply to position controllers
    for (int i = 0; i < 3; i++) {
        pidControllers[i]->SetGains(params.position.P, params.position.I, params.position.D);
    }

    // Apply to attitude controllers
    for (int i = 3; i < 6; i++) {
        pidControllers[i]->SetGains(params.attitude.P, params.attitude.I, params.attitude.D);
    }
}

bool AutoPIDTuner::CheckStability() const
{
    if (performanceHistory.Num() < 10) return true;

    // Check last few performances for violent oscillations or divergence
    float sum = 0.0f;
    float variance = 0.0f;

    // Calculate mean
    for (int i = performanceHistory.Num() - 10; i < performanceHistory.Num(); i++) {
        sum += performanceHistory[i];
    }
    float mean = sum / 10.0f;

    // Calculate variance
    for (int i = performanceHistory.Num() - 10; i < performanceHistory.Num(); i++) {
        float diff = performanceHistory[i] - mean;
        variance += diff * diff;
    }
    variance /= 10.0f;

    // High variance indicates instability
    return variance < 0.5f;  // Threshold can be adjusted
}

void AutoPIDTuner::Reset()
{
    tuningComplete = false;
    currentIteration = 0;
    parameterIndex = 0;
    currentDeltaP = initialDelta;
    currentDeltaI = initialDelta;
    currentDeltaD = initialDelta;
    performanceHistory.Empty();
}