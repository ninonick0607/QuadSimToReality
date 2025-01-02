// QuadUI.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "UI/FlightModeHUD.h"
#include "QuadUI.generated.h"

UENUM(BlueprintType)
enum class EMenuOptions : uint8
{
    StartFlying    UMETA(DisplayName = "Start Flying"),
    MapModel       UMETA(DisplayName = "Map/Model"),
    Settings       UMETA(DisplayName = "Settings"),
    Credits        UMETA(DisplayName = "Credits"),
    Exit           UMETA(DisplayName = "Exit")
};

UCLASS()
class QUADSIMTOREALITY_API UQuadUI : public UUserWidget
{
    GENERATED_BODY()

public:
    virtual void NativeConstruct() override;

protected:
    // Button widgets
    UPROPERTY(meta = (BindWidget))
    class UButton* StartFlyingButton;

    UPROPERTY(meta = (BindWidget))
    class UButton* MapModelButton;
    
    UPROPERTY(meta = (BindWidget))
    class UButton* SettingsButton;

    UPROPERTY(meta = (BindWidget))
    class UButton* CreditsButton;
    
    UPROPERTY(meta = (BindWidget))
    class UButton* ExitButton;

    // Widget class references
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "UI")
    TSubclassOf<UUserWidget> MapModelWidgetClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "UI")
    TSubclassOf<UUserWidget> SettingsWidgetClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "UI")
    TSubclassOf<UUserWidget> CreditsWidgetClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "UI")
    TSubclassOf<UFlightModeHUD> FlightModeHUDClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "UI")
    TSubclassOf<class UGameUI> GameUIClass;

    // Fix signature to match delegate
    UFUNCTION()
    void OnFlightModeSelected(EFlightOptions SelectedMode);
    
    // Button click handlers
    UFUNCTION()
    void OnStartFlyingClicked();

    UFUNCTION()
    void OnMapModelClicked();

    UFUNCTION()
    void OnSettingsClicked();

    UFUNCTION()
    void OnCreditsClicked();

    UFUNCTION()
    void OnExitClicked();

private:
    void BeginCameraTransition();
    void CompleteGameplaySetup(class AQuadPawn* QuadPawn, class APlayerController* PlayerController);
    EFlightOptions StoredFlightMode; 
    // Camera transition properties
    FVector InitialCameraLocation;
    FRotator InitialCameraRotation;
    float CurrentTransitionTime;
    FTimerHandle TransitionTimerHandle;
};