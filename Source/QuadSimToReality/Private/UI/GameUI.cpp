// GameHUD.cpp
#include "UI/GameUI.h"
#include "Components/ProgressBar.h"
#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "UI/CoreUI/UIManager.h"
#include "Kismet/GameplayStatics.h"

void UGameUI::NativeConstruct()
{
    Super::NativeConstruct();

    // Bind button events
    if (SwitchCameraButton)
    {
        SwitchCameraButton->OnClicked.AddDynamic(this, &UGameUI::OnSwitchCameraClicked);
    }

    // Initial update
    UpdateHUDElements();
}

void UGameUI::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
    Super::NativeTick(MyGeometry, InDeltaTime);
    UpdateHUDElements();
}

void UGameUI::UpdateHUDElements()
{
    UQuadDroneController* DroneController = GetDroneController();
    if (!DroneController) return;

    // Get current flight data
    FVector CurrentLocation = DroneController->GetCurrentAltitude();
    FVector CurrentVelocity = DroneController->GetCurrentVelocity();
    float MaxVelocity = DroneController->GetMaxVelocity();
    float CurrentSpeed = CurrentVelocity.Size();

    // Update altitude display
    if (AltitudeBar)
    {
        float NormalizedAltitude = FMath::Clamp(CurrentLocation.Z / MaxDisplayAltitude, 0.0f, 1.0f);
        AltitudeBar->SetPercent(NormalizedAltitude);
    }

    if (AltitudeText)
    {
        // Convert to meters for display
        AltitudeText->SetText(FText::FromString(
            FString::Printf(TEXT("Altitude: %.1f m"), CurrentLocation.Z / 100.0f)));
    }

    // Update velocity display
    if (VelocityBar)
    {
        float NormalizedVelocity = FMath::Clamp(CurrentSpeed / MaxVelocity, 0.0f, 1.0f);
        VelocityBar->SetPercent(NormalizedVelocity);
    }

    if (VelocityText)
    {
        // Convert to m/s for display
        VelocityText->SetText(FText::FromString(
            FString::Printf(TEXT("Velocity:\nX: %.1f\nY: %.1f\nZ: %.1f"),
                CurrentVelocity.X / 100.0f,
                CurrentVelocity.Y / 100.0f,
                CurrentVelocity.Z / 100.0f)));
    }

    // Update flight mode text
    if (FlightModeText)
    {
        EFlightOptions CurrentMode = DroneController->GetCurrentFlightMode();
        FString ModeName = UEnum::GetValueAsString(CurrentMode);
        FlightModeText->SetText(FText::FromString(FString::Printf(TEXT("Mode: %s"), *ModeName)));
    }
}

void UGameUI::OnSwitchCameraClicked()
{
    if (UQuadDroneController* DroneController = GetDroneController())
    {
        DroneController->SwitchCamera();
    }
}

UQuadDroneController* UGameUI::GetDroneController() const
{
    APlayerController* PC = GetOwningPlayer();
    if (!PC) return nullptr;

    APawn* Pawn = PC->GetPawn();
    if (!Pawn) return nullptr;

    // Get the QuadPawn and its controller
    if (AQuadPawn* QuadPawn = Cast<AQuadPawn>(Pawn))
    {
        return QuadPawn->QuadController;
    }

    return nullptr;
}