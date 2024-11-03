// QuadPawn.cpp

#include "QuadPawn.h"
#include "QuadDroneController.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"

#define EPSILON 0.0001f

const FVector start = FVector(0, 0, 1000);

static TArray<FVector> make_test_dests()
{
    const int step = 1000;
    const int z_step = 200;
    TArray<FVector> xyzSetpoint;
    xyzSetpoint.Add(start);
    for (int i = 0; i < 1000; i++)
    {
        bool z = FMath::RandBool();
        bool x = FMath::RandBool();
        bool y = FMath::RandBool();
        FVector last = xyzSetpoint[xyzSetpoint.Num() - 1];
        float z_base = 1000;
        xyzSetpoint.Add(FVector(last.X + (x ? step : -step), last.Y + (y ? step : -step), z ? z_base + z_step : z_base - z_step));
    }
    return xyzSetpoint;
}
AQuadPawn::AQuadPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // Initialize arrays
    Motors.SetNum(4);
    Thrusters.SetNum(4);
    
    const float DroneSize = 12.0f;
    const float ThrusterHeight = 16.0f;
    const FString MotorNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
    const FString ThrusterNames[] = { TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR") };

    // Initialize components
    DroneBody = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneBody"));
    SetRootComponent(DroneBody);

    DroneBody->SetSimulatePhysics(true);
    DroneBody->SetLinearDamping(1.0f);
    DroneBody->SetAngularDamping(1.0f);

    DroneCamMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CamMesh"));
    DroneCamMesh->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
    
    CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
    CameraFPV->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepWorldTransform);

    CameraFPV->SetRelativeLocation(FVector(0, 0, 10));


    const FVector MotorPositions[] = {
        FVector(DroneSize, -DroneSize, ThrusterHeight),   // Front-Left
        FVector(DroneSize, DroneSize, ThrusterHeight),    // Front-Right
        FVector(-DroneSize, -DroneSize, ThrusterHeight),  // Back-Left
        FVector(-DroneSize, DroneSize, ThrusterHeight)    // Back-Right
    };

    // Initialize Rotors
    for (int32 i = 0; i < 4; ++i)
    {
        // Create the motor mesh component
        Motors[i] = CreateDefaultSubobject<UStaticMeshComponent>(*MotorNames[i]);
        Motors[i]->SetupAttachment(DroneBody);
        Motors[i]->SetRelativeLocation(MotorPositions[i]);

        // Optionally set other properties for Motors[i] here

        // Create the thruster component
        Thrusters[i] = CreateDefaultSubobject<UPhysicsThrusterComponent>(*ThrusterNames[i]);
        Thrusters[i]->SetupAttachment(Motors[i]);
        Thrusters[i]->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
        Thrusters[i]->bAutoActivate = true;
        Thrusters[i]->ThrustStrength = 0.0f; // Initialize thrust to zero

        // Initialize the Rotor struct
        Rotors[i] = { Thrusters[i], Motors[i] };
    }

    // SpringArm settings
    SpringArm->TargetArmLength = 200.0f;
    SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->bDoCollisionTest = false;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;

    // Automatically possess pawn for testing
    AutoPossessPlayer = EAutoReceiveInput::Player0;
    
    bWaypointModeSelected = false;

    // Initialize the drone controller
    quadController = new QuadDroneController(this);

    Input_ToggleImguiInput = CreateDefaultSubobject<UInputComponent>(TEXT("Toggle Imgui Input"));
    Input_ToggleImguiInput->BindKey(EKeys::I, IE_Pressed, this, &AQuadPawn::ToggleImguiInput).bExecuteWhenPaused = true;


}

void AQuadPawn::BeginPlay()
{
    Super::BeginPlay();

    for (auto& rotor : Rotors)
    {
        rotor.Thruster->ThrustStrength = 0.0f;
    }
    
    WaypointMode = EWaypointMode::WaitingForModeSelection;
    NewWaypoint = FVector(0.0f, 0.0f, 0.0f);
    ManualWaypoints.Empty();
    // for testing
    this->quadController->AddNavPlan("TestPlan", make_test_dests());
    this->quadController->SetNavPlan("TestPlan");

    for (auto& rotor : Rotors)
    {
        rotor.Thruster->ThrustStrength = 0.0f;
        rotor.Mesh->GetBodyInstance()->SetMassOverride(1.0f);
        rotor.Mesh->GetBodyInstance()->UpdateMassProperties();
    }
    
    float DroneMass = 1.0f; // Adjust this value to something appropriate for the drone size
    DroneBody->GetBodyInstance()->SetMassOverride(DroneMass);   
    DroneBody->GetBodyInstance()->UpdateMassProperties();
    
    quadController->Reset();
    Camera->SetActive(true);
    CameraFPV->SetActive(false);
}

void AQuadPawn::SwitchCamera()
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

void AQuadPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    for (auto& rotor : Rotors)
    {
        rotor.Animate(DeltaTime);
    }

    
    // UpdateAnimation(DeltaTime);
    UpdateControl(DeltaTime);
}

void AQuadPawn::HandleThrustInput(float Value)
{
    if (quadController)
    {
        quadController->HandleThrustInput(Value);
    }
}

void AQuadPawn::HandleYawInput(float Value)
{
    if (quadController)
    {
        quadController->HandleYawInput(Value);
    }
}

void AQuadPawn::HandlePitchInput(float Value)
{
    if (quadController)
    {
        quadController->HandlePitchInput(Value);
    }
}

void AQuadPawn::HandleRollInput(float Value)
{
    if (quadController)
    {
        quadController->HandleRollInput(Value);
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
}

void AQuadPawn::UpdateControl(float DeltaTime)
{
    this->quadController->Update(DeltaTime);
}


void AQuadPawn::InitializeRotors()
{
    const FString motorNames[] = { TEXT("MotorFL"), TEXT("MotorFR"), TEXT("MotorBL"), TEXT("MotorBR") };
    const FString thrusterNames[] = { TEXT("ThrusterFL"), TEXT("ThrusterFR"), TEXT("ThrusterBL"), TEXT("ThrusterBR") };
    const FString socketNames[] = { TEXT("MotorSocketFL"), TEXT("MotorSocketFR"), TEXT("MotorSocketBL"), TEXT("MotorSocketBR") };

    for (int i = 0; i < 4; ++i)
    {
        // Create and attach motor components
        Motors[i] = CreateDefaultSubobject<UStaticMeshComponent>(*motorNames[i]);
        Motors[i]->AttachToComponent(DroneBody, FAttachmentTransformRules::KeepRelativeTransform);
        Motors[i]->SetupAttachment(DroneBody, *socketNames[i]);

        // Create and attach thruster components under the motor
        Thrusters[i] = CreateDefaultSubobject<UPhysicsThrusterComponent>(*thrusterNames[i]);
        Thrusters[i]->AttachToComponent(Motors[i], FAttachmentTransformRules::KeepRelativeTransform);
        Thrusters[i]->SetRelativeRotation(FRotator(-90.0f, 0.0f, 0.0f));
        Thrusters[i]->bAutoActivate = true;
        
    }
}

// Rendering waypoint selection using ImGui
void AQuadPawn::RenderWaypointModeSelection()
{
    ImGui::Begin("Waypoint Mode Selection");
    ImGui::Text("Select a Waypoint Mode");

    if (!bWaypointModeSelected)
    {
        if (ImGui::Button("Random Waypoints"))
        {
            UE_LOG(LogTemp, Log, TEXT("Random Waypoints button pressed"));
            quadController->Reset(); 
            quadController->AddNavPlan("RandomPlan", make_test_dests());
            quadController->SetNavPlan("RandomPlan");
            WaypointMode = EWaypointMode::ReadyToStart;
            bWaypointModeSelected = true;
        }

        if (ImGui::Button("Manual Waypoints"))
        {
            UE_LOG(LogTemp, Log, TEXT("Manual Waypoints button pressed"));
            WaypointMode = EWaypointMode::ManualWaypointInput;
            bWaypointModeSelected = true;
        }
    }

    ImGui::End();
}

// Render manual waypoint input using ImGui
void AQuadPawn::RenderManualWaypointInput()
{
    ImGui::Begin("Manual Waypoints Setup");

    float waypointInput[3] = { static_cast<float>(NewWaypoint.X), static_cast<float>(NewWaypoint.Y), static_cast<float>(NewWaypoint.Z) };

    if (ImGui::InputFloat3("New Waypoint", waypointInput))
    {
        NewWaypoint.X = static_cast<double>(waypointInput[0]);
        NewWaypoint.Y = static_cast<double>(waypointInput[1]);
        NewWaypoint.Z = static_cast<double>(waypointInput[2]);
    }

    if (ImGui::Button("Add Waypoint"))
    {
        ManualWaypoints.Add(NewWaypoint);
    }

    if (ImGui::Button("Clear Waypoints"))
    {
        ManualWaypoints.Empty();
    }

    ImGui::Separator();
    ImGui::Text("Current Waypoints:");
    for (int i = 0; i < ManualWaypoints.Num(); ++i)
    {
        ImGui::BulletText("Waypoint %d: X=%f, Y=%f, Z=%f", i + 1, ManualWaypoints[i].X, ManualWaypoints[i].Y, ManualWaypoints[i].Z);
    }

    ImGui::Separator();
    if (ImGui::Button("Start Drone"))
    {
        if (ManualWaypoints.Num() > 0)
        {
            quadController->Reset(); 
            quadController->AddNavPlan("ManualPlan", ManualWaypoints);
            quadController->SetNavPlan("ManualPlan");
            WaypointMode = EWaypointMode::ReadyToStart;
        }
        else
        {
            ImGui::Text("Please add at least one waypoint before starting the drone.");
        }
    }

    ImGui::End();
}
