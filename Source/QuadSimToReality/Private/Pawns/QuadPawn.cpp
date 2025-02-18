
// QuadPawn.cpp

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Controllers/ZMQController.h"
#include "Core/DroneGlobalState.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

#define EPSILON 0.0001f

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

	// Initialize components
	DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
	SetRootComponent(DroneBody);
	DroneBody->SetSimulatePhysics(true);

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
	
	ImGuiUtil = CreateDefaultSubobject<UImGuiUtil>(TEXT("DroneImGuiUtil"));
	ZMQController = CreateDefaultSubobject<UZMQController>(TEXT("ZMQController"));
	
	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AQuadPawn::BeginPlay()
{
	Super::BeginPlay();
	
	if (!IsValid(this))
	{
		UE_LOG(LogTemp, Error, TEXT("AQuadPawn::BeginPlay: 'this' is not valid?"));
		return;
	}
	if (!QuadController)
	{
		QuadController = NewObject<UQuadDroneController>(this, TEXT("QuadDroneController"));
		QuadController->Initialize(this);
	}
	if (ZMQController)
	{
		ZMQController->Initialize(this, QuadController, ZMQController->GetConfiguration());
		ZMQController->SetDroneID(GetName());
		PawnLocalID = ZMQController->GetConfiguration().DroneID;
	}
	
    UE_LOG(LogTemp, Display, TEXT("QuadPawn BeginPlay: Pawn=%p, Name=%s"), this, *GetName());

	UDroneGlobalState::Get()->RegisterPawn(this);

	if (ZMQController)
	{
		UE_LOG(LogTemp, Display, TEXT("ZMQController updated to use unique DroneID: %s"), *PawnLocalID);
	}

	if (ImGuiUtil)
	{
		ImGuiUtil->Initialize(this, QuadController, FVector::ZeroVector,
				true, true, 100.0f, 600.0f, 45.0f);
	}

	QuadController->ResetPID();
}



void AQuadPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);


	APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC || !PC->IsLocalController())
	{
		return; 
	}

	static uint64 LastDrawFrame = 0;
	uint64 CurrentFrame = GFrameCounter;

	if (LastDrawFrame != CurrentFrame)
	{
		LastDrawFrame = CurrentFrame;

		FString MyDroneID = (ZMQController)
			? ZMQController->GetConfiguration().DroneID 
			: FString("NoZMQ");
		UDroneGlobalState::Get()->DrawDroneManagerWindow(GetWorld(), MyDroneID);
	}

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
