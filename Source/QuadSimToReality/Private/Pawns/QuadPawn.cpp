#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Core/DroneJSONConfig.h"
#include "Engine/Engine.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"

#define EPSILON 0.0001f

AQuadPawn::AQuadPawn()
	: DroneBody(nullptr)
	, SpringArm(nullptr)
	, Camera(nullptr)
	, CameraFPV(nullptr)
	, QuadController(nullptr)
	, Input_ToggleImguiInput(nullptr)
{
	PrimaryActorTick.bCanEverTick = true;

	// Create and configure DroneBody
	DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    RootComponent = DroneBody;
	DroneBody->SetSimulatePhysics(true);

	CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
	CameraFPV->SetupAttachment(DroneBody, TEXT("FPVCam"));  // Attach to FPVCam socket
	CameraFPV->SetRelativeScale3D(FVector(0.1f));
	CameraFPV->SetActive(true);
	
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(DroneBody);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm);

	// Configure SpringArm
	SpringArm->TargetArmLength = 200.f;
	SpringArm->SetRelativeRotation(FRotator(-20.f, 0.f, 0.f));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;

	// Setup propellers and thrusters
	const FString propellerNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
	const FString socketNames[] = { TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR") };

	Propellers.SetNum(4);
	Thrusters.SetNum(4);
	PropellerRPMs.SetNum(4);

	for (int i = 0; i < 4; i++)
	{
		Propellers[i] = CreateDefaultSubobject<UStaticMeshComponent>(*propellerNames[i]);
		Propellers[i]->SetSimulatePhysics(false);
		Propellers[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Propellers[i]->SetupAttachment(DroneBody, *socketNames[i]);

		Thrusters[i] = CreateDefaultSubobject<UThrusterComponent>(
			*FString::Printf(TEXT("Thruster_%s"), *propellerNames[i])
		);
		Thrusters[i]->SetupAttachment(DroneBody, *socketNames[i]);
		Thrusters[i]->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));

		PropellerRPMs[i] = 0.f;

	}

	// Create additional components
	ImGuiUtil = CreateDefaultSubobject<UImGuiUtil>(TEXT("DroneImGuiUtil"));
	
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	SceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
	SceneCapture->SetupAttachment(DroneBody, TEXT("FPVCam")); 
	SceneCapture->CaptureSource = SCS_FinalColorLDR;

	RenderTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("CameraRenderTarget"));
	RenderTarget->InitCustomFormat(128, 128, PF_B8G8R8A8, false);
	RenderTarget->UpdateResourceImmediate(true);
	SceneCapture->CaptureScene(); 
	RenderTarget->UpdateResource();
	SceneCapture->TextureTarget = RenderTarget;

	bIsFPVActive = true;
	bFirstCapture = true;

}

void AQuadPawn::BeginPlay()
{
	Super::BeginPlay();
	
	DroneID = GetName();
	UE_LOG(LogTemp, Display, TEXT("QuadPawn BeginPlay: DroneID set to %s"), *DroneID);
	
	if (!QuadController)
	{
		QuadController = NewObject<UQuadDroneController>(this, TEXT("QuadDroneController"));
		QuadController->Initialize(this);
	}

	UE_LOG(LogTemp, Display, TEXT("QuadPawn BeginPlay: Pawn=%p, Name=%s"), this, *GetName());
	
	if (ImGuiUtil)
	{
		ImGuiUtil->Initialize(this, QuadController);
	}

	QuadController->ResetPID();
}

void AQuadPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateControl(DeltaTime);

	for (int32 i = 0; i < Propellers.Num(); i++)
	{
		if (Propellers[i])
		{
			float DirectionMultiplier = 1.0f;
			if (MotorClockwiseDirections.IsValidIndex(i))
			{
				DirectionMultiplier = MotorClockwiseDirections[i] ? -1.0f : 1.0f;
			}
			float DegreesPerSecond = PropellerRPMs[i] * 6.0f;
            float DeltaRotation = DegreesPerSecond * DeltaTime * DirectionMultiplier;
			Propellers[i]->AddLocalRotation(FRotator(0.f, DeltaRotation, 0.f));
		}
	}

	APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !PC->IsLocalController())
	{
		return;
	}

}

void AQuadPawn::UpdateControl(float DeltaTime)
{
	if (QuadController)
	{	
		QuadController->Update(DeltaTime);
	}
}

void AQuadPawn::SwitchCamera() const
{
	if (CameraFPV->IsActive())
	{
		// Switch to third-person view.
		CameraFPV->SetActive(false);
		Camera->SetActive(true);
	}
	else
	{
		// Switch to first-person view.
		CameraFPV->SetActive(true);
		Camera->SetActive(false);
	}
}

void AQuadPawn::ToggleImguiInput()
{
	UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("ImGui.ToggleInput");
}

void AQuadPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	PlayerInputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadPawn::ToggleImguiInput);
	PlayerInputComponent->BindAction("ReloadJSON", IE_Pressed, this, &AQuadPawn::ReloadJSONConfig);
}

void AQuadPawn::ReloadJSONConfig()
{
	UDroneJSONConfig::Get().ReloadConfig();
}

void AQuadPawn::CaptureCameraImage(TArray<uint8>& OutImageData)
{
	if (!RenderTarget || !SceneCapture) return;

	// Skip first frame (often blank)
	if (bFirstCapture) {
		bFirstCapture = false;
		return;
	}

	// Force GPU flush and scene capture
	FlushRenderingCommands();
	SceneCapture->CaptureScene();

	FTextureRenderTargetResource* RTResource = RenderTarget->GameThread_GetRenderTargetResource();
	if (RTResource)
	{
		// Read pixels with correct flags
		PixelData.Reset();
		FReadSurfaceDataFlags ReadFlags(RCM_UNorm, CubeFace_MAX);
		ReadFlags.SetLinearToGamma(false); // Important for correct color space
        
		RTResource->ReadPixels(PixelData, ReadFlags);

		// Convert BGRA to RGB
		OutImageData.Reset(PixelData.Num() * 3 / 4); // 4 channels -> 3 channels
		for (FColor& Pixel : PixelData)
		{
			OutImageData.Add(Pixel.B);
			OutImageData.Add(Pixel.G);
			OutImageData.Add(Pixel.R);
		}

		// Debug check
		if (OutImageData.Num() > 3 && 
			OutImageData[0] == 0 && OutImageData[1] == 0 && OutImageData[2] == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("First pixel is still black!"));
		}
	}
}