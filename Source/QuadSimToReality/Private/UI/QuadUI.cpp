// QuadUI.cpp
#include "UI/QuadUI.h"
#include "Components/Button.h"
#include "UI/CoreUI/UIManager.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Camera/CameraActor.h"
#include "UI/GameUI.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"

void UQuadUI::NativeConstruct()
{
    Super::NativeConstruct();

    // Bind all button events
    if (StartFlyingButton)
    {
        StartFlyingButton->OnClicked.AddDynamic(this, &UQuadUI::OnStartFlyingClicked);
    }
    
    if (MapModelButton)
    {
        MapModelButton->OnClicked.AddDynamic(this, &UQuadUI::OnMapModelClicked);
    }
    
    if (SettingsButton)
    {
        SettingsButton->OnClicked.AddDynamic(this, &UQuadUI::OnSettingsClicked);
    }
    
    if (CreditsButton)
    {
        CreditsButton->OnClicked.AddDynamic(this, &UQuadUI::OnCreditsClicked);
    }
    
    if (ExitButton)
    {
        ExitButton->OnClicked.AddDynamic(this, &UQuadUI::OnExitClicked);
    }
}

void UQuadUI::OnStartFlyingClicked()
{
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (!UIManager || !FlightModeHUDClass)
    {
        UE_LOG(LogTemp, Warning, TEXT("UIManager or FlightModeHUDClass not found!"));
        return;
    }

    // Create flight mode HUD through UIManager
    UFlightModeHUD* FlightHUD = Cast<UFlightModeHUD>(
        UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Game, FlightModeHUDClass)
    );

    if (FlightHUD)
    {
        FlightHUD->OnFlightModeSelected.AddDynamic(this, &UQuadUI::OnFlightModeSelected);
        UIManager->PopContentFromLayer(EUILayer::Menu);
        UE_LOG(LogTemp, Display, TEXT("FlightModeHUD added to viewport"));
    }
}

void UQuadUI::OnMapModelClicked()
{
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (!UIManager || !MapModelWidgetClass)
        return;

    UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Menu, MapModelWidgetClass);
    UIManager->PopContentFromLayer(EUILayer::Menu);
}

void UQuadUI::OnSettingsClicked()
{
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (!UIManager || !SettingsWidgetClass)
        return;

    UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Menu, SettingsWidgetClass);
    UIManager->PopContentFromLayer(EUILayer::Menu);
}

void UQuadUI::OnCreditsClicked()
{
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (!UIManager || !CreditsWidgetClass)
        return;

    UIManager->PushContentToLayer(GetOwningPlayer(), EUILayer::Menu, CreditsWidgetClass);
    UIManager->PopContentFromLayer(EUILayer::Menu);
}

void UQuadUI::OnExitClicked()
{
    APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
    if (PlayerController)
    {
        UKismetSystemLibrary::QuitGame(GetWorld(), PlayerController, EQuitPreference::Quit, false);
    }
}

void UQuadUI::OnFlightModeSelected(EFlightOptions SelectedMode)
{
    StoredFlightMode = SelectedMode;

    BeginCameraTransition();
}

// In QuadUI.cpp, update BeginCameraTransition():
void UQuadUI::BeginCameraTransition()
{
    APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
    AQuadPawn* QuadPawn = Cast<AQuadPawn>(UGameplayStatics::GetActorOfClass(GetWorld(), AQuadPawn::StaticClass()));
    
    if (!PlayerController || !QuadPawn)
        return;

    // Store initial camera position for interpolation
    ACameraActor* MenuCamera = Cast<ACameraActor>(PlayerController->GetViewTarget());
    if (MenuCamera)
    {
        InitialCameraLocation = MenuCamera->GetActorLocation();
        InitialCameraRotation = MenuCamera->GetActorRotation();
    }

    // Calculate the final camera position based on SpringArm settings
    FVector DroneLocation = QuadPawn->GetActorLocation();
    FRotator DroneRotation = QuadPawn->GetActorRotation();
    
    FRotator SpringArmRotation = DroneRotation + QuadPawn->SpringArm->GetRelativeRotation();
    FVector SpringArmOffset = SpringArmRotation.Vector() * -QuadPawn->SpringArm->TargetArmLength;
    FVector TargetLocation = DroneLocation + SpringArmOffset;
    FRotator TargetRotation = SpringArmRotation;

    // Start the transition timer
    float BlendTime = 2.0f;
    float UpdateInterval = 0.016f;
    CurrentTransitionTime = 0.0f;

    // Ensure the drone doesn't start flying yet
    if (UQuadDroneController* DroneController = QuadPawn->GetQuadController())
    {
        DroneController->SetFlightMode(EFlightOptions::None);
    }

    GetWorld()->GetTimerManager().SetTimer(
        TransitionTimerHandle,
        [this, PlayerController, QuadPawn, TargetLocation, TargetRotation, BlendTime, UpdateInterval]()
        {
            CurrentTransitionTime += UpdateInterval;
            float Alpha = CurrentTransitionTime / BlendTime;
            float SmoothedAlpha = Alpha * Alpha * (3.0f - 2.0f * Alpha);

            if (ACameraActor* CurrentCamera = Cast<ACameraActor>(PlayerController->GetViewTarget()))
            {
                FVector NewLocation = FMath::Lerp(InitialCameraLocation, TargetLocation, SmoothedAlpha);
                FRotator NewRotation = FMath::Lerp(InitialCameraRotation, TargetRotation, SmoothedAlpha);
                
                CurrentCamera->SetActorLocation(NewLocation);
                CurrentCamera->SetActorRotation(NewRotation);
            }

            if (CurrentTransitionTime >= BlendTime)
            {
                GetWorld()->GetTimerManager().ClearTimer(TransitionTimerHandle);
                CompleteGameplaySetup(QuadPawn, PlayerController);
            }
        },
        UpdateInterval,
        true
    );
}
void UQuadUI::CompleteGameplaySetup(AQuadPawn* QuadPawn, APlayerController* PlayerController)
{
    if (!QuadPawn || !PlayerController)
        return;

    // Get the drone controller
    UQuadDroneController* DroneController = QuadPawn->GetQuadController();
    if (!DroneController)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get QuadDroneController in CompleteGameplaySetup"));
        return;
    }

    // Initialize the flight mode directly through the controller
    DroneController->InitializeFlightMode(StoredFlightMode);
    
    // Set up player control
    PlayerController->SetViewTarget(QuadPawn);
    QuadPawn->EnableInput(PlayerController);

    // Configure input mode
    FInputModeGameAndUI InputMode;
    InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    InputMode.SetHideCursorDuringCapture(false);
    PlayerController->SetInputMode(InputMode);
    PlayerController->bShowMouseCursor = true;

    // Create GameUI after camera transition
    UUIManager* UIManager = GetGameInstance()->GetSubsystem<UUIManager>();
    if (UIManager && GameUIClass)
    {
        UIManager->PopContentFromLayer(EUILayer::Menu);
        UIManager->PushContentToLayer(PlayerController, EUILayer::Game, GameUIClass);
    }
}