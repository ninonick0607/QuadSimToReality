#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Core/DroneJSONConfig.h"
#include "EngineUtils.h"

#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h" 
#include "Components/PrimitiveComponent.h" 
#include "GameFramework/Actor.h"        
#include "Core/ThrusterComponent.h"       
#include "UI/ImGuiUtil.h"   
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

	for (TActorIterator<AQuadPawn> ActorItr(GWorld); ActorItr; ++ActorItr)
	{
		if (*ActorItr)
		{
			currentPos = (*ActorItr)->GetActorLocation();
			break;
		}
	}

	xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::startHeight));
	int numLoops = FMath::CeilToInt((DroneWaypointConfig::maxHeight - DroneWaypointConfig::startHeight) / DroneWaypointConfig::heightStep);

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
	xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::maxHeight));

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
	xyzSetpoint.Add(FVector(currentPos.X, currentPos.Y, currentPos.Z + DroneWaypointConfig::startHeight));
	return xyzSetpoint;
}

const FName ObstacleCollisionTag = FName("Obstacle");

AQuadPawn::AQuadPawn()
	: DroneBody(nullptr) // 1
	  , SpringArm(nullptr) // 3
	  , Camera(nullptr) // 4
	  , CameraFPV(nullptr) // 5
	  , WaypointMode(EWaypointMode::WaitingForModeSelection) // 14
	  , NewWaypoint(FVector::ZeroVector) // 16
	  , QuadController(nullptr) // 20
	  , bWaypointModeSelected(false) // 21
      , bHasCollidedWithObstacle(false)
	  , Input_ToggleImguiInput(nullptr)
{
	PrimaryActorTick.bCanEverTick = true;

	// Create and configure DroneBody
	DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    RootComponent = DroneBody;
	DroneBody->SetSimulatePhysics(true);
	DroneBody->SetNotifyRigidBodyCollision(true);
    DroneBody->SetGenerateOverlapEvents(true);

	DroneBody->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName); 
	
	CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
	CameraFPV->SetupAttachment(DroneBody,TEXT("FPVCam"));
	CameraFPV->SetRelativeScale3D(FVector(0.1f));

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
	NavigationComponent = CreateDefaultSubobject<UNavigationComponent>(TEXT("NavigationComponent"));
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
	
	if (!ImGuiUtil)
	{
		ImGuiUtil = NewObject<UImGuiUtil>(this, UImGuiUtil::StaticClass(), TEXT("DroneImGuiUtil"));
		ImGuiUtil->Initialize(this, QuadController);
	}

	// Continue with any other initialization (like binding or logging)
	if (ImGuiUtil)
	{
		UE_LOG(LogTemp, Display, TEXT("ImGuiUtil successfully created and initialized"));
		ImGuiUtil->Initialize(this, QuadController);
	}

	NavigationComponent->SetNavigationPlan(spiralWaypoints());

	// Reset PID controllers
	QuadController->ResetPID();
	DroneBody->OnComponentHit.AddDynamic(this, &AQuadPawn::OnDroneHit);
	ResetCollisionStatus();
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

void AQuadPawn::OnDroneHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, 
						   UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (OtherActor && OtherActor != this)
	{
		UE_LOG(LogTemp, Display, TEXT("Hit detected with: %s"), *OtherActor->GetName());

		if (OtherActor->ActorHasTag(ObstacleCollisionTag))
		{
			// Set the collision flag if not already set
			if (!bHasCollidedWithObstacle)
			{
				bHasCollidedWithObstacle = true;
				UE_LOG(LogTemp, Display, TEXT("%s collided with obstacle: %s"), *GetName(), *OtherActor->GetName());
                
				GetWorld()->GetTimerManager().ClearTimer(CollisionHoldTimerHandle);
				GetWorld()->GetTimerManager().SetTimer(CollisionHoldTimerHandle, this,
					&AQuadPawn::ResetCollisionStatus, CollisionHoldDuration, false);
			}
		}
	}
}

void AQuadPawn::ResetCollisionStatus()
{
	if (bHasCollidedWithObstacle) 
	{
		UE_LOG(LogTemp, Log, TEXT("%s collision status reset."), *GetName());
	}
	bHasCollidedWithObstacle = false;
}
