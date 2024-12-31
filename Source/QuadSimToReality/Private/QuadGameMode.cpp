
// QuadGameMode.cpp
#include "QuadGameMode.h"
#include "UI/QuadUI.h"
#include "UI/CoreUI/UIManager.h"
#include "Kismet/GameplayStatics.h"
#include "Pawns/QuadPawn.h"
#include "Camera/CameraActor.h"
#include "Blueprint/WidgetBlueprintLibrary.h"

AQuadGameMode::AQuadGameMode()
{
	// Set the default pawn class
	DefaultPawnClass = AQuadPawn::StaticClass();
}

void AQuadGameMode::BeginPlay()
{
	Super::BeginPlay();

	// Get UI Manager
	UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
	if (!UIManager || !MainMenuWidgetClass)
		return;

	// Get player controller
	APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	if (!PlayerController)
		return;

	// Find cinematic camera
	ACameraActor* MenuCamera = Cast<ACameraActor>(UGameplayStatics::GetActorOfClass(GetWorld(), ACameraActor::StaticClass()));
	if (MenuCamera)
	{
		PlayerController->SetViewTarget(MenuCamera);
	}

	// Show cursor and set input mode
	PlayerController->bShowMouseCursor = true;
    
	FInputModeUIOnly InputMode;
	InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	PlayerController->SetInputMode(InputMode);

	// Create main menu through UIManager
	UIManager->PushContentToLayer(PlayerController, EUILayer::Menu, MainMenuWidgetClass);
}