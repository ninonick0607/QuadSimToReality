#include "Core/DroneMathUtils.h"
#include "Math/UnrealMathUtility.h"

FVector DroneMathUtils::CalculateDesiredVelocity(const FVector& Error, float MaxVelocity)
{
    return Error.GetSafeNormal() * MaxVelocity;
}

float DroneMathUtils::CalculateDesiredRoll(const FVector& NormalizedError, const FVector& DroneForwardVector, float MaxTilt, float AltitudeThreshold)
{
    FVector HorizontalError = FVector(NormalizedError.X, NormalizedError.Y, 0.0f);
    FVector HorizontalNormalizedError = HorizontalError.GetSafeNormal();

    if (FMath::Abs(NormalizedError.Z) > AltitudeThreshold)
    {
        return 0.0f; // Ignore roll adjustment if altitude error is significa
    }
    else
    {
        float CalculatedRoll = FMath::Atan2(NormalizedError.Y, FVector::DotProduct(HorizontalNormalizedError, DroneForwardVector)) *
                               FMath::RadiansToDegrees(1);
        return FMath::Clamp(CalculatedRoll, -MaxTilt, MaxTilt);
    }
}

float DroneMathUtils::CalculateDesiredPitch(const FVector& NormalizedError, const FVector& DroneForwardVector, float MaxTilt, float AltitudeThreshold)
{
    FVector HorizontalError = FVector(NormalizedError.X, NormalizedError.Y, 0.0f);
    FVector HorizontalNormalizedError = HorizontalError.GetSafeNormal();

    if (FMath::Abs(NormalizedError.Z) > AltitudeThreshold)
    {
        return 0.0f; // Ignore pitch adjustment if altitude error is significant
    }
    else
    {
        float CalculatedPitch = FMath::Atan2(-NormalizedError.X, FVector::DotProduct(HorizontalNormalizedError, DroneForwardVector)) *
                                FMath::RadiansToDegrees(1);
        return FMath::Clamp(CalculatedPitch, -MaxTilt, MaxTilt);
    }
}

float DroneMathUtils::NormalizeAngle(float Angle)
{
    return FMath::Fmod(Angle + 180.0f, 360.0f) - 180.0f;
}

FVector DroneMathUtils::ClampVectorMagnitude(const FVector& Vector, float MaxMagnitude)
{
    float Magnitude = Vector.Size();
    if (Magnitude > MaxMagnitude)
    {
        return Vector.GetSafeNormal() * MaxMagnitude;
    }
    return Vector;
}
