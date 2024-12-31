// FlightModeHUD.cpp
#include "UI/FlightModeHUD.h"
#include "UI/QuadUI.h"
#include "UI/GameUI.h"
#include "Components/Button.h"
#include "UI/CoreUI/UIManager.h"

void UFlightModeHUD::NativeConstruct()
{
	Super::NativeConstruct();

	if (AutoWaypointButton)
	{
		AutoWaypointButton->OnClicked.AddDynamic(this, &UFlightModeHUD::OnAutoWaypointSelected);
	}

	if (JoyStickControlButton)
	{
		JoyStickControlButton->OnClicked.AddDynamic(this, &UFlightModeHUD::OnJoyStickControlSelected);
	}

	if (VelocityControlButton)
	{
		VelocityControlButton->OnClicked.AddDynamic(this, &UFlightModeHUD::OnVelocityControlSelected);
	}

	if (BackButton)
	{
		BackButton->OnClicked.AddDynamic(this, &UFlightModeHUD::OnBackSelected);
	}
}

void UFlightModeHUD::OnAutoWaypointSelected()
{
	NotifyModeSelection(EFlightOptions::AutoWaypoint);
}

void UFlightModeHUD::OnJoyStickControlSelected()
{
	NotifyModeSelection(EFlightOptions::JoyStickControl);
}

void UFlightModeHUD::OnVelocityControlSelected()
{
	NotifyModeSelection(EFlightOptions::VelocityControl);
}

void UFlightModeHUD::OnBackSelected()
{
	UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
	if (!UIManager || !MainMenuClass)
		return;

	// Remove current HUD and return to main menu
	UIManager->PopContentFromLayer(EUILayer::Game);
	UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Menu, MainMenuClass);
}

void UFlightModeHUD::NotifyModeSelection(EFlightOptions SelectedMode)
{
	// First broadcast the selection to handle camera transition
	OnFlightModeSelected.Broadcast(SelectedMode);
    
	// Create the game HUD using UIManager
	UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
	if (UIManager && GameUIClass)
	{
		// Remove flight mode menu
		UIManager->PopContentFromLayer(EUILayer::Game);
        
		// Wait a bit for camera transition before showing game UI
		FTimerHandle TimerHandle;
		GetWorld()->GetTimerManager().SetTimer(TimerHandle, [this, UIManager]()
		{
			// Create game UI after camera transition
			UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Game, GameUIClass);
		}, 2.0f, false); // Match this with your camera transition time
	}
}