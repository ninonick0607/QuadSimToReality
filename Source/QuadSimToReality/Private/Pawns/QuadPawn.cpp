// QuadPawn.cpp

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "EngineUtils.h"
#include "Controllers/ZMQController.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

#define EPSILON 0.0001f
// At the top of QuadPawn.cpp

namespace DroneWaypointConfig 
{
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
    FVector currentPos = FVector::ZeroVector;

    // Find the drone in the world
    for (TActorIterator<AQuadPawn> ActorItr(GWorld); ActorItr; ++ActorItr)
    {
        if (*ActorItr)
        {
            currentPos = (*ActorItr)->GetActorLocation();
            break;
        }
    }

    // First point at starting height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::startHeight));

    // Calculate number of loops needed
    int numLoops = FMath::CeilToInt((DroneWaypointConfig::maxHeight - DroneWaypointConfig::startHeight) 
                                   / DroneWaypointConfig::heightStep);

    // Generate upward spiral
    for (int loop = 0; loop < numLoops; loop++)
    {
        float height = currentPos.Z + DroneWaypointConfig::startHeight + (loop * DroneWaypointConfig::heightStep);

        for (int point = 0; point < DroneWaypointConfig::pointsPerLoop; point++)
        {
            float angle = point * DroneWaypointConfig::angleStep;
            float x = currentPos.X + DroneWaypointConfig::radius * FMath::Cos(angle);
            float y = currentPos.Y + DroneWaypointConfig::radius * FMath::Sin(angle);
            xyzSetpoint.Add(FVector(x, y, height));
        }
    }

    // Add point at max height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::maxHeight));

    // Generate downward spiral (in reverse order)
    for (int loop = numLoops - 1; loop >= 0; loop--)
    {
        float height = currentPos.Z + DroneWaypointConfig::startHeight + (loop * DroneWaypointConfig::heightStep);

        for (int point = DroneWaypointConfig::pointsPerLoop - 1; point >= 0; point--)
        {
            float angle = point * DroneWaypointConfig::angleStep;
            float x = currentPos.X + DroneWaypointConfig::radius * FMath::Cos(angle);
            float y = currentPos.Y + DroneWaypointConfig::radius * FMath::Sin(angle);
            xyzSetpoint.Add(FVector(x, y, height));
        }
    }

    // Final point back at starting height above current position
    xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::startHeight));

    return xyzSetpoint;
}

AQuadPawn::AQuadPawn()
	: DroneBody(nullptr) // 1
	  , DroneCamMesh(nullptr) // 2
	  , SpringArm(nullptr) // 3
	  , Camera(nullptr) // 4
	  , CameraFPV(nullptr) // 5
	  , WaypointMode(EWaypointMode::WaitingForModeSelection) // 14
	  , NewWaypoint(FVector::ZeroVector) // 16
	  , QuadController(nullptr) // 20
	  , bWaypointModeSelected(false) // 21
	  , Input_ToggleImguiInput(nullptr)
{
	PrimaryActorTick.bCanEverTick = true;

	
	const FString propellerNames[] = {TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR")};
	const FString socketNames[] = {
		TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR")
	};
	
	// Initialize arrays
	Propellers.SetNum(4);
	Thrusters.SetNum(4);
	
	for (int i = 0; i < 4; i++)
	{
		// Create propellers
		Propellers[i] = CreateDefaultSubobject<UStaticMeshComponent>(*propellerNames[i]);
		Propellers[i]->SetSimulatePhysics(false);
		Propellers[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Propellers[i]->SetupAttachment(DroneBody, *socketNames[i]);

		Thrusters[i] = CreateDefaultSubobject<UThrusterComponent>(
			*FString::Printf(TEXT("Thruster_%s"), *propellerNames[i])
		);
		Thrusters[i]->SetupAttachment(DroneBody,*socketNames[i]); 
		Thrusters[i]->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));

	}
    // Initialize components
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

	// Initialize ZMQController
	if (!ZMQController)
	{
		ZMQController = NewObject<UZMQController>(this, TEXT("ZMQController"));
        
		FZMQConfiguration Config;
        
		ZMQController->Initialize(this, QuadController, Config);
		ZMQController->RegisterComponent(); 
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

void AQuadPawn::UpdateControl(float DeltaTime)
{
	this->QuadController->Update(DeltaTime);
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
void AQuadPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)  
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	
	PlayerInputComponent->BindAction("ToggleImGui", IE_Pressed, this, &AQuadPawn::ToggleImguiInput);
}