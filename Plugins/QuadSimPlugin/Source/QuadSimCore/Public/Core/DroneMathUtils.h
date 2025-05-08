#pragma once

#include "CoreMinimal.h"

/**
 * Utility class for drone-specific mathematical calculations.
 */
class QUADSIMCORE_API DroneMathUtils
{
public:
    /**
     * Calculates the desired velocity vector based on the position error and maximum velocity.
     * 
     * @param Error The position error vector.
     * @param MaxVelocity The maximum allowable velocity.
     * @return The desired velocity vector.
     */
    static FVector CalculateDesiredVelocity(const FVector& Error, float MaxVelocity);

    /**
     * Calculates the desired roll angle for the drone based on the position error.
     * 
     * @param NormalizedError The normalized error vector.
     * @param DroneForwardVector The forward vector of the drone.
     * @param MaxTilt The maximum allowable tilt angle in degrees.
     * @param AltitudeThreshold The altitude error threshold for roll adjustments.
     * @return The desired roll angle in degrees.
     */
    static float CalculateDesiredRoll(const FVector& NormalizedError, const FVector& DroneForwardVector, float MaxTilt, float AltitudeThreshold);

    /**
     * Calculates the desired pitch angle for the drone based on the position error.
     * 
     * @param NormalizedError The normalized error vector.
     * @param DroneForwardVector The forward vector of the drone.
     * @param MaxTilt The maximum allowable tilt angle in degrees.
     * @param AltitudeThreshold The altitude error threshold for pitch adjustments.
     * @return The desired pitch angle in degrees.
     */
    static float CalculateDesiredPitch(const FVector& NormalizedError, const FVector& DroneForwardVector, float MaxTilt, float AltitudeThreshold);

    /**
     * Normalizes an angle to the range [-180, 180].
     * 
     * @param Angle The input angle in degrees.
     * @return The normalized angle in degrees.
     */ 
    static float NormalizeAngle(float Angle);

    /**
     * Clamps a vector's magnitude to a maximum value while preserving its direction.
     * 
     * @param Vector The input vector.
     * @param MaxMagnitude The maximum allowable magnitude.
     * @return The clamped vector.
     */
    static FVector ClampVectorMagnitude(const FVector& Vector, float MaxMagnitude);
};
