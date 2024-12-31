#include "Core/DroneGlobalState.h"
#include "UI/FlightModeHUD.h" // Add this include
#include "Controllers/QuadDroneController.h" // Include here instead of in header

DroneGlobalState::DroneGlobalState()
    : DesiredVelocity(FVector::ZeroVector)
    , BoundController(nullptr)
{
}

DroneGlobalState::~DroneGlobalState()
{
}

// void DroneGlobalState::SetDesiredVelocity(const FVector& NewVelocity)
// {
//     DesiredVelocity = NewVelocity;
//     if (BoundController)
//     {
//         BoundController->SetDesiredVelocity(NewVelocity);
//         BoundController->SetFlightMode(UQuadDroneController::UFlightModeHUD::VelocityControl);
//     }
// }

void DroneGlobalState::BindController(UQuadDroneController* Controller)
{
    BoundController = Controller;
}

void DroneGlobalState::UnbindController()
{
    BoundController = nullptr;
}