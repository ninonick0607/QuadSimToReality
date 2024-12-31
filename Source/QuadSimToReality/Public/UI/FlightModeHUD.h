#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/Button.h"
#include "FlightModeHUD.generated.h"

UENUM(BlueprintType)
enum class EFlightOptions : uint8
{
	AutoWaypoint,
	JoyStickControl,
	VelocityControl
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnFlightModeSelectedSignature, EFlightOptions, SelectedMode);

UCLASS()
class QUADSIMTOREALITY_API UFlightModeHUD : public UUserWidget
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable, Category = "Events")
	FOnFlightModeSelectedSignature OnFlightModeSelected;

protected:
	virtual void NativeConstruct() override;

	// UI Elements
	UPROPERTY(BlueprintReadWrite, meta = (BindWidget))
	class UButton* AutoWaypointButton;

	UPROPERTY(BlueprintReadWrite, meta = (BindWidget))
	class UButton* JoyStickControlButton;

	UPROPERTY(BlueprintReadWrite, meta = (BindWidget))
	class UButton* VelocityControlButton;

	UPROPERTY(BlueprintReadWrite, meta = (BindWidget))
	class UButton* BackButton;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "UI")
	TSubclassOf<class UQuadUI> MainMenuClass;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "UI")
	TSubclassOf<class UGameUI> GameUIClass;

private:
	UFUNCTION()
	void OnAutoWaypointSelected();

	UFUNCTION()
	void OnJoyStickControlSelected();

	UFUNCTION()
	void OnVelocityControlSelected();

	UFUNCTION()
	void OnBackSelected();

	void NotifyModeSelection(EFlightOptions SelectedMode);
};