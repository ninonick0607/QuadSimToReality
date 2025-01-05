// FlightModeHUD.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "FlightModeHUD.generated.h"

UENUM(BlueprintType)
enum class EFlightOptions : uint8
{
	None             UMETA(DisplayName = "None"),
	AutoWaypoint     UMETA(DisplayName = "Auto Waypoint"),
	JoyStickControl  UMETA(DisplayName = "JoyStick Control"),
	VelocityControl  UMETA(DisplayName = "Velocity Control")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnFlightModeSelected, EFlightOptions, SelectedMode);

UCLASS()
class QUADSIMTOREALITY_API UFlightModeHUD : public UUserWidget
{
	GENERATED_BODY()

public:
	virtual void NativeConstruct() override;

	UPROPERTY(BlueprintAssignable, Category = "Events")
	FOnFlightModeSelected OnFlightModeSelected;
	
	UFUNCTION(BlueprintCallable, Category = "Flight Mode")
	void NotifyModeSelection(EFlightOptions SelectedMode);

protected:
	// Buttons
	UPROPERTY(meta = (BindWidget))
	class UButton* AutoWaypointButton;

	UPROPERTY(meta = (BindWidget))
	class UButton* JoyStickControlButton;

	UPROPERTY(meta = (BindWidget))
	class UButton* VelocityControlButton;

	UPROPERTY(meta = (BindWidget))
	class UButton* BackButton;

	// Button handlers
	UFUNCTION()
	void OnAutoWaypointSelected();

	UFUNCTION()
	void OnJoyStickControlSelected();

	UFUNCTION()
	void OnVelocityControlSelected();

	UFUNCTION()
	void OnBackSelected();

\
	// Class references
	UPROPERTY(EditDefaultsOnly, Category = "UI")
	TSubclassOf<class UUserWidget> MainMenuClass;

	UPROPERTY(EditDefaultsOnly, Category = "UI")
	TSubclassOf<class UUserWidget> GameUIClass;
};