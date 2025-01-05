// DroneControlPanel.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "UI/FlightModeHUD.h"
#include "DroneControlPanel.generated.h"

class AQuadPawn;
class UQuadDroneController;
class QuadPIDController;
class UButton;
class USlider;
class UTextBlock;
class UEditableTextBox;
class UVerticalBox;
class UCheckBox;
class UBorder;

USTRUCT()
struct FPIDControllerSet
{
    GENERATED_BODY()

    QuadPIDController* XPID;
    QuadPIDController* YPID;
    QuadPIDController* ZPID;
    QuadPIDController* RollPID;
    QuadPIDController* PitchPID;
    QuadPIDController* YawPID;

    FPIDControllerSet()
        : XPID(nullptr), YPID(nullptr), ZPID(nullptr)
        , RollPID(nullptr), PitchPID(nullptr), YawPID(nullptr)
    {}

    FPIDControllerSet(
        QuadPIDController* InXPID, QuadPIDController* InYPID, QuadPIDController* InZPID,
        QuadPIDController* InRollPID, QuadPIDController* InPitchPID, QuadPIDController* InYawPID)
        : XPID(InXPID), YPID(InYPID), ZPID(InZPID)
        , RollPID(InRollPID), PitchPID(InPitchPID), YawPID(InYawPID)
    {}
};

UCLASS()
class QUADSIMTOREALITY_API UDroneControlPanel : public UUserWidget
{
    GENERATED_BODY()

public:
    virtual void NativeConstruct() override;
    virtual void NativeDestruct() override;

    void InitializeDroneControl(
        AQuadPawn* InPawn, 
        UQuadDroneController* InController,
        const FPIDControllerSet& AutoWaypointPIDs,
        const FPIDControllerSet& VelocityPIDs,
        const FPIDControllerSet& JoyStickPIDs
    );
    virtual bool Initialize() override;

    void UpdateDisplay();

protected:
    
    UPROPERTY(meta = (BindWidget))
    UVerticalBox* MainContainer;

    UPROPERTY(meta = (BindWidget))
    USlider* MaxVelocitySlider;

    UPROPERTY(meta = (BindWidget))
    USlider* MaxTiltAngleSlider;

    UPROPERTY(meta = (BindWidget))
    UCheckBox* CollisionSphereCheckbox;

    UPROPERTY(meta = (BindWidget))
    UCheckBox* WaypointCheckbox;

    UPROPERTY(meta = (BindWidget))
    UButton* SwitchCameraButton;

    UPROPERTY(meta = (BindWidget))
    UButton* ResetHighButton;

    UPROPERTY(meta = (BindWidget))
    UButton* ResetOriginButton;

    // PID Control Groups
    UPROPERTY(meta = (BindWidget))
    UVerticalBox* PositionPIDContainer;

    UPROPERTY(meta = (BindWidget))
    UVerticalBox* AttitudePIDContainer;

    // Thrust Bars
    UPROPERTY(meta = (BindWidget))
    class UProgressBar* ThrustFL;

    UPROPERTY(meta = (BindWidget))
    class UProgressBar* ThrustFR;
    
    UPROPERTY(meta = (BindWidget))
    class UProgressBar* ThrustBL;
    
    UPROPERTY(meta = (BindWidget))
    class UProgressBar* ThrustBR;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ThrustFLText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ThrustFRText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ThrustBLText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ThrustBRText;

    // Event Handlers
    UFUNCTION()
    void OnMaxVelocityChanged(float Value);

    UFUNCTION()
    void OnMaxTiltAngleChanged(float Value);

    UFUNCTION()
    void OnCollisionSphereToggled(bool bIsChecked);

    UFUNCTION()
    void OnWaypointToggled(bool bIsChecked);

    UFUNCTION()
    void OnSwitchCameraClicked();

    UFUNCTION()
    void OnResetHighClicked();

    UFUNCTION()
    void OnResetOriginClicked();


private:
    void SetupPIDControls();
    void UpdatePIDValues();
    void CreatePIDGroup(UVerticalBox* Container, const FString& Label, QuadPIDController* Controller);
    void UpdateActivePIDSet(EFlightOptions FlightMode);

    UPROPERTY()
    AQuadPawn* DronePawn;

    UPROPERTY()
    UQuadDroneController* DroneController;

    // PID controller sets for different flight modes
    FPIDControllerSet AutoWaypointPIDSet;
    FPIDControllerSet VelocityPIDSet;
    FPIDControllerSet JoyStickPIDSet;
    
    // Current active PID set
    FPIDControllerSet* CurrentPIDSet;
};