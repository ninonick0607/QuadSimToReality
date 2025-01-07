// GameUI.cpp
#include "UI/GameUI.h"
#include "Components/ProgressBar.h"
#include "Components/TextBlock.h"
#include "Components/Image.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "UI/DroneControlPanel.h"
#include "UI/CoreUI/UIManager.h"    

void UGameUI::NativeConstruct()
{
    Super::NativeConstruct();

    if (AltitudeIndicator)
    {
        UMaterial* BaseMaterial = Cast<UMaterial>(StaticLoadObject(
            UMaterial::StaticClass(), 
            nullptr, 
            TEXT("/Game/Blueprints/UI/Images/M_AltIndicator")
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
            PlayerInputComponent->BindAction("ToggleDroneControl", IE_Pressed, this, &UGameUI::ToggleDroneControlPanel);
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

void UGameUI::ToggleDroneControlPanel()
{
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (!UIManager || !DroneControlPanelClass)
        return;

    if (ActiveDronePanel)
    {
        // If panel is already open, remove it
        ActiveDronePanel->RemoveFromParent();
        ActiveDronePanel = nullptr;
        UE_LOG(LogTemp, Warning, TEXT("DroneControlPanel removed from viewport"));
        return;
    }

    // Create and initialize DroneControlPanel
    if (UQuadDroneController* DroneController = GetDroneController())
    {
        if (AQuadPawn* QuadPawn = Cast<AQuadPawn>(DroneController->dronePawn))
        {
            ActiveDronePanel = Cast<UDroneControlPanel>(
                UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Modal, DroneControlPanelClass)
            );
            
            if (ActiveDronePanel)
            {
                FFullPIDSet* AutoSet = DroneController->GetPIDSet(EFlightOptions::AutoWaypoint);
                FFullPIDSet* VelSet  = DroneController->GetPIDSet(EFlightOptions::VelocityControl);
                FFullPIDSet* JoySet  = DroneController->GetPIDSet(EFlightOptions::JoyStickControl);
                
                FPIDControllerSet AutoWaypointPIDs(
                    AutoSet ? AutoSet->XPID : nullptr,
                    AutoSet ? AutoSet->YPID : nullptr,
                    AutoSet ? AutoSet->ZPID : nullptr,
                    AutoSet ? AutoSet->RollPID : nullptr,
                    AutoSet ? AutoSet->PitchPID : nullptr,
                    AutoSet ? AutoSet->YawPID : nullptr
                );

                FPIDControllerSet VelocityPIDs(
                    VelSet ? VelSet->XPID : nullptr,
                    VelSet ? VelSet->YPID : nullptr,
                    VelSet ? VelSet->ZPID : nullptr,
                    VelSet ? VelSet->RollPID : nullptr,
                    VelSet ? VelSet->PitchPID : nullptr,
                    VelSet ? VelSet->YawPID : nullptr
                );

                FPIDControllerSet JoyStickPIDs(
                    JoySet ? JoySet->XPID : nullptr,
                    JoySet ? JoySet->YPID : nullptr,
                    JoySet ? JoySet->ZPID : nullptr,
                    JoySet ? JoySet->RollPID : nullptr,
                    JoySet ? JoySet->PitchPID : nullptr,
                    JoySet ? JoySet->YawPID : nullptr
                );
                // Now pass them into DroneControlPanel
                ActiveDronePanel->InitializeDroneControl(
                    QuadPawn,
                    DroneController,
                    AutoWaypointPIDs,
                    VelocityPIDs,
                    JoyStickPIDs
                );
                
                UE_LOG(LogTemp, Warning, TEXT("DroneControlPanel created and added to viewport"));
            }
        }
    }
}


void UGameUI::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
    Super::NativeTick(MyGeometry, InDeltaTime);
    UpdateHUDElements();
}

FReply UGameUI::NativeOnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent)
{
    if (InKeyEvent.IsControlDown() && InKeyEvent.GetKey() == EKeys::D)
    {
        ToggleDroneControlPanel();
        return FReply::Handled();
    }

    return FReply::Unhandled();
}

void UGameUI::UpdateHUDElements()
{
    UQuadDroneController* DroneController = GetDroneController();
    if (!DroneController || !AltitudeScrollMaterial) return;

    APawn* OwnerPawn = GetOwningPlayerPawn();
    if (!OwnerPawn) return;
    
    FVector CurrentLocation = DroneController->GetCurrentPosition();
    float ScrollOffset = FMath::Fmod(CurrentLocation.Z / 500.0f, 1.0f);
    AltitudeScrollMaterial->SetScalarParameterValue("ScrollAmount", -ScrollOffset);
    
    FVector ForwardVector = OwnerPawn->GetActorForwardVector(); 
    FVector CurrentVelocity = DroneController->GetCurrentVelocity();

    float ForwardVelocity = FVector::DotProduct(CurrentVelocity, ForwardVector);
    float ForwardVelocityMeters = ForwardVelocity / 100.0f;

    float MaxVelocity = DroneController->GetMaxVelocity();
    float CurrentSpeed = CurrentVelocity.Size();

    if (VelocityBar)
    {
        float NormalizedVelocity = FMath::Clamp(CurrentSpeed / MaxVelocity, 0.0f, 1.0f);
        VelocityBar->SetPercent(NormalizedVelocity);
    }

    if (AltitudeText)
    {
        AltitudeText->SetText(FText::FromString(
            FString::Printf(TEXT(" %.1fm"), CurrentLocation.Z / 100.0f)));
    }

    if (VelocityText)
    {
        VelocityText->SetText(FText::FromString(
            FString::Printf(TEXT("Forward Velocity: %.1f m/s"), ForwardVelocityMeters)));
    }

    if (FlightModeText)
    {
        FlightModeText->SetText(FText::FromString(
            FString::Printf(TEXT("Mode: %s"), 
            *UEnum::GetValueAsString(DroneController->GetCurrentFlightMode()))));
    }

    if (ActiveDronePanel)
    {
        ActiveDronePanel->UpdateDisplay();
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