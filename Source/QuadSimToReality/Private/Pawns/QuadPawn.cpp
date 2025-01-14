// QuadPawn.cpp

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

#define EPSILON 0.0001f
// At the top of QuadPawn.cpp

namespace {
	static constexpr float startHeight = 1000.0f;
	static constexpr float maxHeight = 10000.0f;
	static constexpr float radius = 3000.0f;
	static constexpr float heightStep = 500.0f;
	static constexpr int32 pointsPerLoop = 8;
	static constexpr float angleStep = 2.0f * PI / pointsPerLoop;

}

const FVector start = FVector(0, 0, 1000);


static TArray<FVector> spiralWaypoints()
{
    TArray<FVector> xyzSetpoint;

    // Get current position
    AQuadPawn* drone = nullptr;
    FVector currentPos = FVector::ZeroVector;

    // Find the drone in the world
    for (TActorIterator<AQuadPawn> ActorItr(GWorld); ActorItr; ++ActorItr)
    {
        drone = *ActorItr;
        if (drone)
        {
            currentPos = drone->GetActorLocation();
            break;
        }
    }

    // First point at starting height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + startHeight));

    // Calculate number of loops needed
    int numLoops = FMath::CeilToInt((maxHeight - startHeight) / heightStep);

    // Generate upward spiral
    for (int loop = 0; loop < numLoops; loop++)
    {
        float height = currentPos.Z + startHeight + (loop * heightStep);

        for (int point = 0; point < pointsPerLoop; point++)
        {
            float angle = point * angleStep;
            float x = currentPos.X + radius * FMath::Cos(angle);
            float y = currentPos.Y + radius * FMath::Sin(angle);
            xyzSetpoint.Add(FVector(x, y, height));
        }
    }

    // Add point at max height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + maxHeight));

    // Generate downward spiral (in reverse order)
    for (int loop = numLoops - 1; loop >= 0; loop--)
    {
        float height = currentPos.Z + startHeight + (loop * heightStep);

        for (int point = pointsPerLoop - 1; point >= 0; point--)
        {
            float angle = point * angleStep;
            float x = currentPos.X + radius * FMath::Cos(angle);
            float y = currentPos.Y + radius * FMath::Sin(angle);
            xyzSetpoint.Add(FVector(x, y, height));
        }
    }

    // Final point back at starting height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + startHeight));

    return xyzSetpoint;
}

void AQuadPawn::InitializeMotors()
{
	const FString motorNames[] = {TEXT("MotorMeshFL"), TEXT("MotorMeshFR"), TEXT("MotorMeshBL"), TEXT("MotorMeshBR")};
	const FString thrusterNames[] = {TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR")};
	const FString socketNames[] = {
		TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR")
	};

	// Initialize arrays
	Thrusters.SetNum(4);
	MotorMeshes.SetNum(4);

	// Create motor meshes first (they attach to sockets)
	MotorMeshFL = CreateDefaultSubobject<UStaticMeshComponent>(FName(*motorNames[0]));
	MotorMeshFR = CreateDefaultSubobject<UStaticMeshComponent>(FName(*motorNames[1]));
	MotorMeshBL = CreateDefaultSubobject<UStaticMeshComponent>(FName(*motorNames[2]));
	MotorMeshBR = CreateDefaultSubobject<UStaticMeshComponent>(FName(*motorNames[3]));

	MotorMeshes = { MotorMeshFL, MotorMeshFR, MotorMeshBL, MotorMeshBR };

	// Create thrusters (they attach to meshes)
	ThrusterFL = CreateDefaultSubobject<UDroneMotorComponent>(FName(*thrusterNames[0]));
	ThrusterFR = CreateDefaultSubobject<UDroneMotorComponent>(FName(*thrusterNames[1]));
	ThrusterBL = CreateDefaultSubobject<UDroneMotorComponent>(FName(*thrusterNames[2]));
	ThrusterBR = CreateDefaultSubobject<UDroneMotorComponent>(FName(*thrusterNames[3]));

	Thrusters = { ThrusterFL, ThrusterFR, ThrusterBL, ThrusterBR };

	// Setup components
	for (int32 i = 0; i < 4; ++i)
	{
		// Setup mesh on socket
		MotorMeshes[i]->SetupAttachment(DroneBody, *socketNames[i]);
		MotorMeshes[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// Setup thruster as child of mesh
		Thrusters[i]->SetupAttachment(MotorMeshes[i]);
		Thrusters[i]->SetAssociatedMesh(MotorMeshes[i]);
	}
}
AQuadPawn::AQuadPawn()
	: DroneBody(nullptr)
	, DroneCamMesh(nullptr)
	, SpringArm(nullptr)
	, Camera(nullptr)
	, CameraFPV(nullptr)
	, WaypointMode(EWaypointMode::WaitingForModeSelection)
	, NewWaypoint(FVector::ZeroVector)
	, QuadController(nullptr)
	, bWaypointModeSelected(false)
	, Input_ToggleImguiInput(nullptr)
{
	PrimaryActorTick.bCanEverTick = true;
    
	// Create root component
	DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
	SetRootComponent(DroneBody);
	DroneBody->SetSimulatePhysics(true);

	// Create camera components
	DroneCamMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CamMesh"));
	DroneCamMesh->SetupAttachment(DroneBody);

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(DroneBody);
    
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm);
    
	CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
	CameraFPV->SetupAttachment(DroneBody);
	CameraFPV->SetRelativeLocation(FVector(5.5f, 0.0f, -6.0f));
	CameraFPV->SetRelativeScale3D(FVector(0.1f));

	// Initialize motors and thrusters
	InitializeMotors();
    
	// Configure SpringArm
	SpringArm->TargetArmLength = 200.0f;
	SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AQuadPawn::BeginPlay()
{
    Super::BeginPlay();

    // Initialize QuadController with this pawn
    if (!QuadController)
    {
        QuadController = NewObject<UQuadDroneController>(this, TEXT("QuadDroneController"));
        QuadController->Initialize(this);
    }

    // Add spiral waypoints navigation plan
    this->QuadController->AddNavPlan("TestPlan", spiralWaypoints());
    this->QuadController->SetNavPlan("TestPlan");
	
    // Reset PID controllers
    QuadController->ResetPID();
}
void AQuadPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateControl(DeltaTime);
}
void AQuadPawn::SwitchCamera() const 
{
	if (CameraFPV->IsActive())
	{
		// enable 3rd person
		CameraFPV->SetActive(false);
		Camera->SetActive(true);
		DroneCamMesh->SetHiddenInGame(false);
	}
	else
	{
		// enable 1st person
		CameraFPV->SetActive(true);
		Camera->SetActive(false);
		DroneCamMesh->SetHiddenInGame(true);
	}
}
void AQuadPawn::ToggleImguiInput()  
{
	UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("ImGui.ToggleInput");
}
void AQuadPawn::HandleThrustInput(float Value)  
{
	if (QuadController)
	{
		QuadController->HandleThrustInput(Value);
	}
}
void AQuadPawn::HandleYawInput(float Value)  
{
	if (QuadController)
	{
		QuadController->HandleYawInput(Value);
	}
}
void AQuadPawn::HandlePitchInput(float Value)  
{
	if (QuadController)
	{
		QuadController->HandlePitchInput(Value);
	}
}
void AQuadPawn::HandleRollInput(float Value)  
{
	if (QuadController)
	{
		QuadController->HandleRollInput(Value);
	}
}
void AQuadPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)  
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Bind axis mappings for controller input to wrapper functions
	PlayerInputComponent->BindAxis("Thrust", this, &AQuadPawn::HandleThrustInput);
	PlayerInputComponent->BindAxis("Yaw", this, &AQuadPawn::HandleYawInput);
	PlayerInputComponent->BindAxis("Pitch", this, &AQuadPawn::HandlePitchInput);
	PlayerInputComponent->BindAxis("Roll", this, &AQuadPawn::HandleRollInput);

	PlayerInputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadPawn::ToggleImguiInput);
}
void AQuadPawn::UpdateControl(float DeltaTime)
{
	this->QuadController->Update(DeltaTime);
}

void AQuadPawn::LockToOnlyPitch()
{
	if (DroneBody)
	{
		// Get body instance
		FBodyInstance* BodyInstance = DroneBody->GetBodyInstance();
		if (!BodyInstance) return;

		// Lock all degrees of freedom first
		BodyInstance->bLockXTranslation = true;
		BodyInstance->bLockYTranslation = true;
		BodyInstance->bLockZTranslation = true;
        
		// Lock specific rotations, leaving pitch (Y) unlocked
		BodyInstance->bLockXRotation = true;  // Lock Roll
		BodyInstance->bLockYRotation = false; // Keep Pitch unlocked
		BodyInstance->bLockZRotation = true;  // Lock Yaw

		// Update the constraints
		DroneBody->SetSimulatePhysics(true);
		DroneBody->UpdateComponentToWorld();
	}
}

void AQuadPawn::UnlockAllMovement()
{
	if (DroneBody)
	{
		// Get body instance
		FBodyInstance* BodyInstance = DroneBody->GetBodyInstance();
		if (!BodyInstance) return;

		// Unlock all translations
		BodyInstance->bLockXTranslation = false;
		BodyInstance->bLockYTranslation = false;
		BodyInstance->bLockZTranslation = false;
        
		// Unlock all rotations
		BodyInstance->bLockXRotation = false;
		BodyInstance->bLockYRotation = false;
		BodyInstance->bLockZRotation = false;

		// Update the constraints
		DroneBody->SetSimulatePhysics(true);
		DroneBody->UpdateComponentToWorld();
	}
}

