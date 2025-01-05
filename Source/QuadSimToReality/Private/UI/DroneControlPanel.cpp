// DroneControlPanel.cpp
#include "UI/DroneControlPanel.h"
#include "Components/VerticalBox.h"
#include "Components/Button.h"
#include "Components/Slider.h"
#include "Components/CheckBox.h"
#include "Components/TextBlock.h"
#include "Components/ProgressBar.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"


void UDroneControlPanel::NativeConstruct()
{
    Super::NativeConstruct();
}

void UDroneControlPanel::NativeDestruct()
{
    // Unbind all delegates
    if (MaxVelocitySlider)
    {
        MaxVelocitySlider->OnValueChanged.RemoveAll(this);
    }

    if (MaxTiltAngleSlider)
    {
        MaxTiltAngleSlider->OnValueChanged.RemoveAll(this);
    }

    if (CollisionSphereCheckbox)
    {
        CollisionSphereCheckbox->OnCheckStateChanged.RemoveAll(this);
    }

    if (WaypointCheckbox)
    {
        WaypointCheckbox->OnCheckStateChanged.RemoveAll(this);
    }

    if (SwitchCameraButton)
    {
        SwitchCameraButton->OnClicked.RemoveAll(this);
    }

    if (ResetHighButton)
    {
        ResetHighButton->OnClicked.RemoveAll(this);
    }

    if (ResetOriginButton)
    {
        ResetOriginButton->OnClicked.RemoveAll(this);
    }

    Super::NativeDestruct();
}



bool UDroneControlPanel::Initialize()
{
    bool Success = Super::Initialize();
    if (!Success)
        return false;
    
    // Set initial position and size
    SetPositionInViewport(FVector2D(100, 100));
    SetDesiredSizeInViewport(FVector2D(400, 600));

    if (MaxVelocitySlider)
    {
        MaxVelocitySlider->OnValueChanged.AddDynamic(this, &UDroneControlPanel::OnMaxVelocityChanged);
    }

    if (MaxTiltAngleSlider)
    {
        MaxTiltAngleSlider->OnValueChanged.AddDynamic(this, &UDroneControlPanel::OnMaxTiltAngleChanged);
    }

    if (CollisionSphereCheckbox)
    {
        CollisionSphereCheckbox->OnCheckStateChanged.AddDynamic(this, &UDroneControlPanel::OnCollisionSphereToggled);
    }

    if (WaypointCheckbox)
    {
        WaypointCheckbox->OnCheckStateChanged.AddDynamic(this, &UDroneControlPanel::OnWaypointToggled);
    }

    if (SwitchCameraButton)
    {
        SwitchCameraButton->OnClicked.AddDynamic(this, &UDroneControlPanel::OnSwitchCameraClicked);
    }

    if (ResetHighButton)
    {
        ResetHighButton->OnClicked.AddDynamic(this, &UDroneControlPanel::OnResetHighClicked);
    }

    if (ResetOriginButton)
    {
        ResetOriginButton->OnClicked.AddDynamic(this, &UDroneControlPanel::OnResetOriginClicked);
    }

    return true;
}

void UDroneControlPanel::InitializeDroneControl(
    AQuadPawn* InPawn, 
    UQuadDroneController* InController,
    const FPIDControllerSet& AutoWaypointPIDs,
    const FPIDControllerSet& VelocityPIDs,
    const FPIDControllerSet& JoyStickPIDs)
{
    DronePawn = InPawn;
    DroneController = InController;

    // Store PID sets
    AutoWaypointPIDSet = AutoWaypointPIDs;
    VelocityPIDSet = VelocityPIDs;
    JoyStickPIDSet = JoyStickPIDs;

    // Set initial values based on current controller state
    if (DroneController)
    {
        if (MaxVelocitySlider)
            MaxVelocitySlider->SetValue(DroneController->GetMaxVelocity());
        if (MaxTiltAngleSlider)
            MaxTiltAngleSlider->SetValue(DroneController->GetMaxAngle());
        
        // Set up initial PID set based on current flight mode
        UpdateActivePIDSet(DroneController->GetCurrentFlightMode());
    }

    SetupPIDControls();
    UpdateDisplay();
}

void UDroneControlPanel::UpdateDisplay()
{
    if (!DroneController || !DronePawn)
        return;

    TArray<float> ThrustValues = DroneController->GetThrusts();
    float CurrentThrustFL = ThrustValues[0];
    float CurrentThrustFR = ThrustValues[1];
    float CurrentThrustBL = ThrustValues[2];
    float CurrentThrustBR = ThrustValues[3];

    float minThrust = 0.f;
    float maxThrust = 600.f;

    if (ThrustFL)
    {
        float normalizedFL = FMath::Clamp((CurrentThrustFL - minThrust) / (maxThrust - minThrust), 0.f, 1.f);
        ThrustFL->SetPercent(normalizedFL);
    }
    if (ThrustFR)
    {
        float normalizedFR = FMath::Clamp((CurrentThrustFR - minThrust) / (maxThrust - minThrust), 0.f, 1.f);
        ThrustFR->SetPercent(normalizedFR);
    }
    if (ThrustBL)
    {
        float normalizedBL = FMath::Clamp((CurrentThrustBL - minThrust) / (maxThrust - minThrust), 0.f, 1.f);
        ThrustBL->SetPercent(normalizedBL);
    }
    if (ThrustBR)
    {
        float normalizedBR = FMath::Clamp((CurrentThrustBR - minThrust) / (maxThrust - minThrust), 0.f, 1.f);
        ThrustBR->SetPercent(normalizedBR);
    }

    if (ThrustFLText)
    {
        ThrustFLText->SetText(FText::FromString(FString::Printf(TEXT("FL: %.1f"), CurrentThrustFL)));
    }
    if (ThrustFRText)
    {
        ThrustFRText->SetText(FText::FromString(FString::Printf(TEXT("FR: %.1f"), CurrentThrustFR)));
    }
    if (ThrustBLText)
    {
        ThrustBLText->SetText(FText::FromString(FString::Printf(TEXT("BL: %.1f"), CurrentThrustBL)));
    }
    if (ThrustBRText)
    {
        ThrustBRText->SetText(FText::FromString(FString::Printf(TEXT("BR: %.1f"), CurrentThrustBR)));
    }

    UpdatePIDValues();
}


void UDroneControlPanel::UpdateActivePIDSet(EFlightOptions FlightMode)
{
    switch (FlightMode)
    {
    case EFlightOptions::AutoWaypoint:
        CurrentPIDSet = &AutoWaypointPIDSet;
        break;
    case EFlightOptions::VelocityControl:
        CurrentPIDSet = &VelocityPIDSet;
        break;
    case EFlightOptions::JoyStickControl:
        CurrentPIDSet = &JoyStickPIDSet;
        break;
    default:
        CurrentPIDSet = &AutoWaypointPIDSet;
        break;
    }

    SetupPIDControls();
}
void UDroneControlPanel::SetupPIDControls()
{
    if (!PositionPIDContainer || !AttitudePIDContainer || !CurrentPIDSet)
        return;

    // Clear existing controls
    PositionPIDContainer->ClearChildren();
    AttitudePIDContainer->ClearChildren();

    // Create PID control groups
    CreatePIDGroup(PositionPIDContainer, "X Position", CurrentPIDSet->XPID);
    CreatePIDGroup(PositionPIDContainer, "Y Position", CurrentPIDSet->YPID);
    CreatePIDGroup(PositionPIDContainer, "Z Position", CurrentPIDSet->ZPID);
    
    CreatePIDGroup(AttitudePIDContainer, "Roll", CurrentPIDSet->RollPID);
    CreatePIDGroup(AttitudePIDContainer, "Pitch", CurrentPIDSet->PitchPID);
    CreatePIDGroup(AttitudePIDContainer, "Yaw", CurrentPIDSet->YawPID);
}

void UDroneControlPanel::CreatePIDGroup(UVerticalBox* Container, const FString& Label, QuadPIDController* Controller)
{
    // Create PID control group dynamically
    // This would include sliders and text boxes for P, I, and D values
    // Implementation details would depend on your specific UMG widget blueprint
}

void UDroneControlPanel::OnMaxVelocityChanged(float Value)
{
    if (DroneController)
        DroneController->SetMaxVelocity(Value);
}

void UDroneControlPanel::OnMaxTiltAngleChanged(float Value)
{
    if (DroneController)
        DroneController->SetMaxAngle(Value);
}

void UDroneControlPanel::OnCollisionSphereToggled(bool bIsChecked)
{
    if (DroneController)
        DroneController->SetDebugDrawCollisionSphere(bIsChecked);
}

void UDroneControlPanel::OnWaypointToggled(bool bIsChecked)
{
    if (DroneController)
        DroneController->SetDebugDrawWaypoint(bIsChecked);
}

void UDroneControlPanel::OnSwitchCameraClicked()
{
    if (DronePawn)
        DronePawn->SwitchCamera();
}

void UDroneControlPanel::OnResetHighClicked()
{
    if (DroneController)
        DroneController->ResetDroneHigh();
}

void UDroneControlPanel::OnResetOriginClicked()
{
    if (DroneController)
        DroneController->ResetDroneOrigin();
}

void UDroneControlPanel::UpdatePIDValues()
{
    // Update displayed PID values based on current controllers
    // Implementation would update the UI elements created in SetupPIDControls
}


