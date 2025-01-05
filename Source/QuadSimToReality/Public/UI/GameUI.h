// GameUI.h
#pragma once

#include "CoreMinimal.h"
#include "DroneControlPanel.h"
#include "Blueprint/UserWidget.h"
#include "GameUI.generated.h"

UCLASS()
class QUADSIMTOREALITY_API UGameUI : public UUserWidget
{
	GENERATED_BODY()

public:
	virtual void NativeConstruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;
	virtual FReply NativeOnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent) override;

protected:
	// Input handling
	UFUNCTION()
	void ToggleInputMode();

	UPROPERTY(meta = (BindWidget))
	class UImage* AltitudeIndicator;

	UPROPERTY()
	class UMaterialInstanceDynamic* AltitudeScrollMaterial;

	// Velocity indicator
	UPROPERTY(meta = (BindWidget))
	class UProgressBar* VelocityBar;

	// Text displays
	UPROPERTY(meta = (BindWidget))
	class UTextBlock* AltitudeText;

	UPROPERTY(meta = (BindWidget))
	class UTextBlock* VelocityText;

	UPROPERTY(meta = (BindWidget))
	class UTextBlock* FlightModeText;

	// Buttons
	UPROPERTY(meta = (BindWidget))
	class UButton* SwitchCameraButton;

	// Configuration
	UPROPERTY(EditDefaultsOnly, Category = "UI|Display")
	float MaxDisplayAltitude = 10000.0f;
	
	UPROPERTY(EditDefaultsOnly, Category = "UI")
	TSubclassOf<UDroneControlPanel> DroneControlPanelClass;

	UPROPERTY()
	UDroneControlPanel* ActiveDronePanel = nullptr;
	
	UFUNCTION()
	void ToggleDroneControlPanel();

private:
	// Button handlers
	UFUNCTION()
	void OnSwitchCameraClicked();

	// Helper functions
	void UpdateHUDElements();
	class UQuadDroneController* GetDroneController() const;
	
};