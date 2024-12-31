// GameHUD.h
#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "UI/FlightModeHUD.h"
#include "GameUI.generated.h"

UCLASS()
class QUADSIMTOREALITY_API UGameUI : public UUserWidget
{
	GENERATED_BODY()

public:
	virtual void NativeConstruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

protected:
	// Progress Bars
	UPROPERTY(meta = (BindWidget))
	class UProgressBar* AltitudeBar;

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

	// UPROPERTY(meta = (BindWidget))
	// class UButton* ReturnToMenuButton;

	// Config
	UPROPERTY(EditDefaultsOnly, Category = "UI|Display")
	float MaxDisplayAltitude = 10000.0f;  // Maximum altitude for progress bar (in cm)

private:
	// Button handlers
	UFUNCTION()
	void OnSwitchCameraClicked();

	// UFUNCTION()
	// void OnReturnToMenuClicked();

	// Helper functions
	void UpdateHUDElements();
	class UQuadDroneController* GetDroneController() const;
};