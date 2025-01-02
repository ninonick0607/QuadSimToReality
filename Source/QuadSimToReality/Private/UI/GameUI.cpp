// GameUI.cpp
#include "UI/GameUI.h"
#include "Components/ProgressBar.h"
#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Components/Image.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"

void UGameUI::NativeConstruct()
{
    Super::NativeConstruct();

    if (AltitudeIndicator)
    {
        UMaterial* BaseMaterial = Cast<UMaterial>(StaticLoadObject(
            UMaterial::StaticClass(), 
            nullptr, 
            TEXT("/Game/Blueprints/UI/Images/M_AltIndicator")  // Updated path
        ));

        if (BaseMaterial)   
        {
            AltitudeScrollMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, this);
            AltitudeIndicator->SetBrushFromMaterial(AltitudeScrollMaterial);
        }
    }
    // Set up input mode
    if (APlayerController* PC = GetOwningPlayer())
    {
        FInputModeGameAndUI InputMode;
        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        InputMode.SetHideCursorDuringCapture(false);
        PC->SetInputMode(InputMode);
        PC->bShowMouseCursor = true;

        // Bind input toggle
        if (UInputComponent* PlayerInputComponent = PC->InputComponent)
        {
            PlayerInputComponent->BindAction("ToggleInput", IE_Pressed, this, &UGameUI::ToggleInputMode);
        }
    }

    UpdateHUDElements();
}

void UGameUI::ToggleInputMode()
{
    APlayerController* PC = GetOwningPlayer();
    if (!PC) return;

    static bool bIsGameOnly = false;
    bIsGameOnly = !bIsGameOnly;

    if (bIsGameOnly)
    {
        FInputModeGameOnly InputMode;
        PC->SetInputMode(InputMode);
        PC->bShowMouseCursor = false;
    }
    else
    {
        FInputModeGameAndUI InputMode;
        InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        InputMode.SetHideCursorDuringCapture(false);
        PC->SetInputMode(InputMode);
        PC->bShowMouseCursor = true;
    }
}

void UGameUI::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
    Super::NativeTick(MyGeometry, InDeltaTime);
    UpdateHUDElements();
}

void UGameUI::UpdateHUDElements()
{
    UQuadDroneController* DroneController = GetDroneController();
    if (!DroneController || !AltitudeScrollMaterial) return;

    FVector CurrentLocation = DroneController->GetCurrentAltitude();
    float ScrollOffset = FMath::Fmod(CurrentLocation.Z / 500.0f, 1.0f);
    AltitudeScrollMaterial->SetScalarParameterValue("ScrollAmount", ScrollOffset);
    FVector CurrentVelocity = DroneController->GetCurrentVelocity();
    float MaxVelocity = DroneController->GetMaxVelocity();
    float CurrentSpeed = CurrentVelocity.Size();

    // Update velocity display
    if (VelocityBar)
    {
        float NormalizedVelocity = FMath::Clamp(CurrentSpeed / MaxVelocity, 0.0f, 1.0f);
        VelocityBar->SetPercent(NormalizedVelocity);
    }

    // Update text displays
    if (AltitudeText)
    {
        AltitudeText->SetText(FText::FromString(
            FString::Printf(TEXT("Altitude: %.1f m"), CurrentLocation.Z / 100.0f)));
    }

    if (VelocityText)
    {
        VelocityText->SetText(FText::FromString(
            FString::Printf(TEXT("Velocity:\nX: %.1f\nY: %.1f\nZ: %.1f"),
                CurrentVelocity.X / 100.0f,
                CurrentVelocity.Y / 100.0f,
                CurrentVelocity.Z / 100.0f)));
    }

    if (FlightModeText)
    {
        FlightModeText->SetText(FText::FromString(
            FString::Printf(TEXT("Mode: %s"), 
            *UEnum::GetValueAsString(DroneController->GetCurrentFlightMode()))));
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

    if (AQuadPawn* QuadPawn = Cast<AQuadPawn>(Pawn))
    {
        return QuadPawn->QuadController;
    }

    return nullptr;
}