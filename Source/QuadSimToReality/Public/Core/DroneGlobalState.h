// DroneGlobalState.h
#pragma once
#include "UI/FlightModeHUD.h" // Add this include
#include "CoreMinimal.h"

class UQuadDroneController;

class QUADSIMTOREALITY_API DroneGlobalState
{
public:
    // Remove the public constructor declaration since it's private
    static DroneGlobalState& Get()
    {
        static DroneGlobalState instance;
        return instance;
    }

    // Getter for velocity
    const FVector& GetDesiredVelocity() const { return DesiredVelocity; }

    // Declare these functions here, define them in the cpp file
    void SetDesiredVelocity(const FVector& NewVelocity);
    void BindController(UQuadDroneController* Controller);
    void UnbindController();

private:
    // Private constructor declaration only
    DroneGlobalState();
    ~DroneGlobalState();

    // Prevent copying
    DroneGlobalState(const DroneGlobalState&) = delete;
    DroneGlobalState& operator=(const DroneGlobalState&) = delete;

    FVector DesiredVelocity;
    UQuadDroneController* BoundController;
};