#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Core/DroneJSONConfig.h"
#include "EngineUtils.h"

#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PrimitiveComponent.h" 
#include "GameFramework/Actor.h"        
#include "Core/ThrusterComponent.h"       
#include "UI/ImGuiUtil.h"
#include "Components/ChildActorComponent.h"
#include "Controllers/ROS2Controller.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

#define EPSILON 0.0001f
// At the top of QuadPawn.cpp

namespace DroneWaypointConfig
{
	static constexpr float startHeight = 500.0f;
	static constexpr float maxHeight = 1000.0f;
	static constexpr float radius = 1000.0f;
	static constexpr float heightStep = 100.0f;
	static constexpr int32 pointsPerLoop = 8;
	static constexpr float angleStep = 2.0f * PI / pointsPerLoop;
}

const FVector start = FVector(0, 0, 1000);

// Generate a repeated figure-8 path after an initial ascent
static TArray<FVector> spiralWaypoints(const FVector& startPos)
{
    TArray<FVector> waypoints;
    // Initial ascent to specified start height
    FVector basePos = startPos;
    const float initialAltitude = DroneWaypointConfig::startHeight;
    const float targetZ = basePos.Z + initialAltitude;
    waypoints.Add(FVector(basePos.X, basePos.Y, targetZ));

    // Figure-8 parameters
    // Increase resolution for smoother curves
    const int32 numPoints = 360;
    const float twoPi = 2.0f * PI;
    // Dimensions: roughly a 10m x 10m box (cm units)
    const float width = DroneWaypointConfig::radius * 0.5f; // ~5m half-width
    const float height = width;                            // symmetrical loops

    // Generate waypoints along the figure-8 curve at constant altitude
    for (int32 i = 0; i <= numPoints; ++i)
    {
        float t = twoPi * static_cast<float>(i) / static_cast<float>(numPoints);
        float x = basePos.X + width * FMath::Sin(t);
        float y = basePos.Y + height * FMath::Sin(2.0f * t);
        waypoints.Add(FVector(x, y, targetZ));
    }
    return waypoints;
}

// Expose figure-8 waypoint generator for the pawn

const FName ObstacleCollisionTag = FName("Obstacle");
TArray<FVector> AQuadPawn::GenerateFigureEightWaypoints() const
{
    return spiralWaypoints(GetActorLocation());
}

AQuadPawn::AQuadPawn()
	: DroneBody(nullptr)
	, SpringArm(nullptr)
	, Camera(nullptr)
	, CameraFPV(nullptr)
	, CameraGroundTrack(nullptr)
	, QuadController(nullptr)
	, ImGuiUtil(nullptr)
	, WaypointMode(EWaypointMode::WaitingForModeSelection)
	, NewWaypoint(FVector::ZeroVector)
	, bHasCollidedWithObstacle(false)
	, CurrentCameraMode(ECameraMode::ThirdPerson)
	, bWaypointModeSelected(false)
{
	PrimaryActorTick.bCanEverTick = true;

    // Skeletal mesh for drone body (physics & visuals)
    DroneBody = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("DroneBody"));
    RootComponent = DroneBody;
    DroneBody->SetSimulatePhysics(true);
    DroneBody->SetNotifyRigidBodyCollision(true);
    DroneBody->SetGenerateOverlapEvents(true);
    DroneBody->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	
	CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
	CameraFPV->SetupAttachment(DroneBody,TEXT("FPVCam"));
	CameraFPV->SetRelativeScale3D(FVector(0.1f));
	CameraFPV->bAutoActivate = true; 

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(DroneBody);
	SpringArm->TargetArmLength = 200.f;
	SpringArm->SetRelativeRotation(FRotator(-20.f, 0.f, 0.f));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName); 
	Camera->bAutoActivate = false; 
	
	CameraGroundTrack = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraGroundTrack"));
	CameraGroundTrack->bAutoActivate = false;
	
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

	ImGuiUtil = CreateDefaultSubobject<UImGuiUtil>(TEXT("DroneImGuiUtil"));
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	NavigationComponent = CreateDefaultSubobject<UNavigationComponent>(TEXT("NavigationComponent"));
    // Child Actor Component for ROS2Controller
    ROS2ControllerComponent = CreateDefaultSubobject<UChildActorComponent>(TEXT("ROS2ControllerComponent"));
    ROS2ControllerComponent->SetupAttachment(RootComponent);
    ROS2ControllerComponent->SetChildActorClass(AROS2Controller::StaticClass());
	
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
    if (ImGuiUtil)
    {
        ImGuiUtil->Initialize(this, QuadController);
    }

	
        // Reset PID controllers
        QuadController->ResetPID();
        // Collision events binding
        if (DroneBody)
        {
            DroneBody->OnComponentHit.AddDynamic(this, &AQuadPawn::OnDroneHit);
        }
        // Collision handling via NotifyHit override; skip AddDynamic binding
	
	ResetCollisionStatus();

	Camera->SetActive(false);
	CameraFPV->SetActive(true);
	CameraGroundTrack->SetActive(false);
	CurrentCameraMode = ECameraMode::FPV;
}

void AQuadPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateControl(DeltaTime);

	for (int32 i = 0; i < Propellers.Num(); i++)
	{
		if (Propellers[i])
		{
			float CurrentThrustVal = QuadController->GetCurrentThrustOutput(i);
			PropellerRPMs[i] = FMath::Abs(CurrentThrustVal) * 1;
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

	UpdateGroundCameraTracking();
	if (bHasCollidedWithObstacle)
	{
		float CurrentTime = GetWorld()->GetTimeSeconds();
		if (CurrentTime - LastCollisionTime > CollisionTimeout)
		{
			bHasCollidedWithObstacle = false;
			UE_LOG(LogTemp, Display, TEXT("%s collision status cleared due to inactivity."), *GetName());
		}
	}
}

void AQuadPawn::UpdateControl(float DeltaTime)
{
	if (QuadController)
	{	
		QuadController->Update(DeltaTime);
	}

	if (NavigationComponent)
	{
		NavigationComponent->UpdateNavigation(GetActorLocation());
		FVector NextGoal = NavigationComponent->GetCurrentSetpoint();
		if (QuadController)
		{
			QuadController->SetDestination(NextGoal);
		}
	}
}

void AQuadPawn::SwitchCamera()
{
	if (!Camera || !CameraFPV || !CameraGroundTrack)
	{
		UE_LOG(LogTemp, Warning, TEXT("SwitchCamera: One or more camera components are missing!"));
		return;
	}

	Camera->SetActive(false);
	CameraFPV->SetActive(false);
	CameraGroundTrack->SetActive(false);

	switch (CurrentCameraMode)
	{
	case ECameraMode::ThirdPerson:
		CurrentCameraMode = ECameraMode::FPV;
		CameraFPV->SetActive(true);
		UE_LOG(LogTemp, Log, TEXT("Camera Mode: FPV"));
		break;

	case ECameraMode::FPV:
		CurrentCameraMode = ECameraMode::GroundTrack;
		CameraGroundTrack->SetActive(true);
		ResetGroundCameraPosition();
		UE_LOG(LogTemp, Log, TEXT("Camera Mode: Ground Track"));
		break;

	case ECameraMode::GroundTrack:
		CurrentCameraMode = ECameraMode::ThirdPerson;
		Camera->SetActive(true);
		UE_LOG(LogTemp, Log, TEXT("Camera Mode: Third Person"));
		break;
	}
}


void AQuadPawn::ResetGroundCameraPosition()
{
	if (!CameraGroundTrack || !DroneBody) return;

	const float GroundOffsetDistance = 200.0f; // 5 meters

	FVector DroneLocation = GetActorLocation();
	FRotator DroneYawRotation(0, GetActorRotation().Yaw, 0);

	FVector RightVector = UKismetMathLibrary::GetRightVector(DroneYawRotation);

	FVector GroundPos = FVector(DroneLocation.X, DroneLocation.Y, 10.0f);
	FVector CameraTargetPosition = GroundPos + RightVector * GroundOffsetDistance;

	CameraGroundTrack->SetWorldLocation(CameraTargetPosition);

	UpdateGroundCameraTracking();
}

void AQuadPawn::UpdateGroundCameraTracking()
{
	if (CurrentCameraMode == ECameraMode::GroundTrack && CameraGroundTrack && CameraGroundTrack->IsActive() && DroneBody)
	{
		FVector CameraLocation = CameraGroundTrack->GetComponentLocation();
		FVector DroneLocation = GetActorLocation(); 

		if (FVector::DistSquaredXY(CameraLocation, DroneLocation) < 1.0f) 
		{
			return;
		}

		FRotator LookAtRotation = UKismetMathLibrary::FindLookAtRotation(CameraLocation, DroneLocation);

		FRotator CurrentRotation = CameraGroundTrack->GetComponentRotation();
		FRotator TargetRotation = FMath::RInterpTo(CurrentRotation, LookAtRotation, GetWorld()->GetDeltaSeconds(), 10.0f); // Adjust interp speed
		CameraGroundTrack->SetWorldRotation(TargetRotation);

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

// Component hit callback
void AQuadPawn::OnDroneHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
    if (OtherActor && OtherActor != this && OtherActor->ActorHasTag(ObstacleCollisionTag))
    {
        LastCollisionTime = GetWorld()->GetTimeSeconds();
        if (!bHasCollidedWithObstacle)
        {
            bHasCollidedWithObstacle = true;
            UE_LOG(LogTemp, Display, TEXT("%s collided with obstacle: %s"), *GetName(), *OtherActor->GetName());
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

float AQuadPawn::GetMass()
{
	return DroneBody ? DroneBody->GetMass() : 0.0f;
}