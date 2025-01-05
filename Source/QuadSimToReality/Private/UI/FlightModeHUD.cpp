// FlightModeHUD.cpp
#include "UI/FlightModeHUD.h"
#include "UI/QuadUI.h"
#include "Pawns/QuadPawn.h"
#include "Components/Button.h"
#include "Kismet/GameplayStatics.h"
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
	// Only broadcast the selection - don't initialize flight mode yet
	OnFlightModeSelected.Broadcast(SelectedMode);
    
	// Handle UI transition
	UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
	if (!UIManager) return;
    
	UIManager->PopContentFromLayer(EUILayer::Game);
    
	// GameUI will be created after camera transition completes in QuadUI
}