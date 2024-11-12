// AutoPIDTuner.h
#pragma once

#include "CoreMinimal.h"
#include "QuadPIDConroller.h"

struct PIDGains {
    float P, I, D;
    PIDGains(float p = 0.0f, float i = 0.0f, float d = 0.0f) : P(p), I(i), D(d) {}
};

struct TuningParameters {
    PIDGains position;   // X, Y, Z gains
    PIDGains attitude;   // Roll, Pitch, Yaw gains
    float fitness;       // How well these parameters perform

    TuningParameters() : fitness(-FLT_MAX) {}
};

class QUADSIMTOREALITY_API AutoPIDTuner {
public:
    AutoPIDTuner();

    void StartTuning(QuadPIDController* xPID, QuadPIDController* yPID, QuadPIDController* zPID,
        QuadPIDController* rollPID, QuadPIDController* pitchPID, QuadPIDController* yawPID);

    void UpdateTuning(const FVector& currentPosition, const FVector& targetPosition,
        const FRotator& currentRotation, const FRotator& targetRotation,
        float deltaTime);

    bool IsTuningComplete() const { return tuningComplete; }
    void Reset();

    // Configuration
    void SetIterationLimit(int limit) { maxIterations = limit; }
    void SetConvergenceThreshold(float threshold) { convergenceThreshold = threshold; }

    // Get best parameters found so far
    const TuningParameters& GetBestParameters() const { return bestParameters; }

private:
    // Tuning state
    bool tuningComplete;
    int currentIteration;
    float currentDeltaP;
    float currentDeltaI;
    float currentDeltaD;
    int parameterIndex;  // Which parameter we're currently tuning (0-5 for P,I,D of position/attitude)

    // Configuration
    int maxIterations;
    float convergenceThreshold;
    float initialDelta;

    // Reference to controllers being tuned
    QuadPIDController* pidControllers[6];  // X, Y, Z, Roll, Pitch, Yaw

    // Best parameters found
    TuningParameters bestParameters;
    TuningParameters currentParameters;

    // Performance history for stability analysis
    TArray<float> performanceHistory;

    float EvaluatePerformance(const FVector& currentPosition, const FVector& targetPosition,
        const FRotator& currentRotation, const FRotator& targetRotation);

    void UpdateParameter(int index, float delta);
    void ApplyParameters(const TuningParameters& params);
    bool CheckStability() const;
};
