
#include "Utility/ObstacleManager.h"
#include "Core/DroneJSONConfig.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Pawns/QuadPawn.h"

const FName ObstacleManager_ObstacleTag = FName("Obstacle");

AObstacleManager::AObstacleManager() {
    PrimaryActorTick.bCanEverTick = true;
    
    VisualMarker = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualMarker"));
    RootComponent = VisualMarker;
    VisualMarker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    VisualMarker->SetVisibility(false); // Hide by default
    
    const auto& Config = UDroneJSONConfig::Get().Config;
    OuterBoundarySize = Config.ObstacleParams.OuterBoundarySize;
    InnerBoundarySize = Config.ObstacleParams.InnerBoundarySize;
    ObstacleSpawnHeight = Config.ObstacleParams.SpawnHeight;
    
    SpawnedGoal = nullptr;
}

void AObstacleManager::BeginPlay() {
    Super::BeginPlay();
    
    VisualizeSpawnBoundaries(true);
}

void AObstacleManager::VisualizeSpawnBoundaries(bool bPersistentLines) {
    if (!GetWorld()) return;
    
    FVector CenterPoint = GetActorLocation();
    
    float HalfOuterSize = OuterBoundarySize * 0.5f;
    float HalfInnerSize = InnerBoundarySize * 0.5f;
    
    FVector OuterExtent(HalfOuterSize, HalfOuterSize, ObstacleSpawnHeight);
    FVector InnerExtent(HalfInnerSize, HalfInnerSize, ObstacleSpawnHeight);
    
    float Duration = bPersistentLines ? -1.0f : 5.0f;
    
    DrawDebugBox(
        GetWorld(),
        CenterPoint,
        OuterExtent,
        FColor(0, 0, 255, 128),
        bPersistentLines,
        Duration,
        0,
        18.0f 
    );
    
    // Draw the inner boundary - red with increased alpha and thickness
    DrawDebugBox(
        GetWorld(),
        CenterPoint,
        InnerExtent,
        FColor(255, 0, 0, 128),  
        bPersistentLines,
        Duration,
        0,
        18.0f 
    );
    
}

FVector AObstacleManager::GetRandomInnerPoint() {
    FVector CenterPoint = GetActorLocation();
    float HalfInnerSize = InnerBoundarySize * 0.5f;
    
    // Random position within inner boundary
    float X = FMath::RandRange(-HalfInnerSize, HalfInnerSize);
    float Y = FMath::RandRange(-HalfInnerSize, HalfInnerSize);
    
    // Return center-relative point
    return CenterPoint + FVector(X, Y, ObstacleSpawnHeight);
}

AActor* AObstacleManager::SpawnObstacle() {
    if (!ObstacleClass || !GetWorld()) {
        UE_LOG(LogTemp, Warning, TEXT("No obstacle class set!"));
        return nullptr;
    }
    
    // Get random position within inner boundary
    FVector SpawnLocation = GetRandomInnerPoint();
    
    UE_LOG(LogTemp, Display, TEXT("Trying to spawn obstacle at %s (Inner Boundary=%f)"), 
           *SpawnLocation.ToString(), InnerBoundarySize);
    
    FRotator SpawnRotation(0.0f, FMath::RandRange(0.0f, 360.0f), 0.0f);
    
    // Spawn parameters
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    
    AActor* Obstacle = GetWorld()->SpawnActor<AActor>(ObstacleClass, 
                                                     SpawnLocation, 
                                                     SpawnRotation, 
                                                     SpawnParams);
    if (Obstacle)
    {
        UStaticMeshComponent* MeshComp = Obstacle->FindComponentByClass<UStaticMeshComponent>();
        if (MeshComp)
        {
            MeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            MeshComp->SetCollisionResponseToAllChannels(ECR_Block);
        }
        Obstacle->Tags.Add(ObstacleManager_ObstacleTag);
        UE_LOG(LogTemp, Log, TEXT("Spawned obstacle %s with tag '%s'"), *Obstacle->GetName(), *ObstacleManager_ObstacleTag.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to spawn obstacle actor!"));
    }
    return Obstacle;
}

AActor* AObstacleManager::SpawnGoal(EGoalPosition Position) {
    if (!GoalClass || !GetWorld()) {
        UE_LOG(LogTemp, Warning, TEXT("No goal class set!"));
        return nullptr;
    }
    
    FVector CenterPoint = GetActorLocation();
    float HalfOuterSize = OuterBoundarySize * 0.5f;
    UE_LOG(LogTemp, Display, TEXT("Goal Actor Position: %f %f %f"), CenterPoint.X, CenterPoint.Y, CenterPoint.Z);

    // Default to a random position if specified
    if (Position == EGoalPosition::Random) {
        Position = static_cast<EGoalPosition>(FMath::RandRange(0, 3));
    }
    
    // Calculate spawn location based on selected boundary face
    FVector SpawnLocation = CenterPoint;
    FRotator SpawnRotation(0.0f, 0.0f, 0.0f);
    
    switch (Position) {
        case EGoalPosition::Front:
            SpawnLocation.X += HalfOuterSize;
            SpawnRotation.Yaw = 180.0f;
            break;
            
        case EGoalPosition::Back:
            SpawnLocation.X -= HalfOuterSize;
            SpawnRotation.Yaw = 0.0f;
            break;
            
        case EGoalPosition::Left:
            SpawnLocation.Y += HalfOuterSize;
            SpawnRotation.Yaw = 270.0f;
            break;
            
        case EGoalPosition::Right:
            SpawnLocation.Y -= HalfOuterSize;
            SpawnRotation.Yaw = 90.0f;
            break;
            
        default:
            break;
    }
    
    // Set height
    SpawnLocation.Z = ObstacleSpawnHeight;
    
    // Spawn parameters
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;

    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    
    // Create goal actor using the class directly
    AActor* Goal = GetWorld()->SpawnActor<AActor>(GoalClass, 
                                                 SpawnLocation, 
                                                 SpawnRotation, 
                                                 SpawnParams);
    
    return Goal;
}

void AObstacleManager::CreateObstacles(int32 NumObstacles, EGoalPosition GoalPos) {
    // Clear any existing obstacles first
    ClearObstacles();
    
    // Spawn requested number of obstacles
    for (int32 i = 0; i < NumObstacles; ++i) {
        AActor* NewObstacle = SpawnObstacle();
        if (NewObstacle) {
            SpawnedObstacles.Add(NewObstacle);
        }
    }
    
    // Handle Random goal position here, before spawning the goal
    EGoalPosition ActualGoalPos = GoalPos;
    if (ActualGoalPos == EGoalPosition::Random) {
        ActualGoalPos = static_cast<EGoalPosition>(FMath::RandRange(0, 3));
    }
    
    // Spawn goal with the actual position
    SpawnedGoal = SpawnGoal(ActualGoalPos);
    
    // Move drone to opposite of the actual goal position
    MoveDroneToOppositeOfGoal(ActualGoalPos);
    
    VisualizeSpawnBoundaries(true);

    UE_LOG(LogTemp, Display, TEXT("Created %d obstacles and 1 goal, drone placed opposite"), NumObstacles);
}
void AObstacleManager::ClearObstacles() {
    // Clear any existing debug drawings first
    FlushPersistentDebugLines(GetWorld());
    
    // Destroy all spawned obstacles
    for (AActor* Obstacle : SpawnedObstacles) {
        if (Obstacle) {
            Obstacle->Destroy();
        }
    }
    SpawnedObstacles.Empty();
    
    // Destroy goal
    if (SpawnedGoal) {
        SpawnedGoal->Destroy();
        SpawnedGoal = nullptr;
    }
    
    UE_LOG(LogTemp, Display, TEXT("Cleared all obstacles and goal"));
}

EGoalPosition AObstacleManager::GetOppositePosition(EGoalPosition Position) {
    switch (Position) {
    case EGoalPosition::Front:
        return EGoalPosition::Back;
    case EGoalPosition::Back:
        return EGoalPosition::Front;
    case EGoalPosition::Left:
        return EGoalPosition::Right;
    case EGoalPosition::Right:
        return EGoalPosition::Left;
    default:
        return EGoalPosition::Back; // Default opposite for Random
    }
}

FVector AObstacleManager::GetPositionLocation(EGoalPosition Position) {
    FVector CenterPoint = GetActorLocation();
    float HalfOuterSize = OuterBoundarySize * 0.5f;
    
    FVector Location = CenterPoint;
    
    switch (Position) {
    case EGoalPosition::Front:
        Location.X += HalfOuterSize;
        break;
    case EGoalPosition::Back:
        Location.X -= HalfOuterSize;
        break;
    case EGoalPosition::Left:
        Location.Y += HalfOuterSize;
        break;
    case EGoalPosition::Right:
        Location.Y -= HalfOuterSize;
        break;
    default:
        break;
    }
    
    Location.Z = ObstacleSpawnHeight;
    return Location;
}

void AObstacleManager::MoveDroneToOppositeOfGoal(EGoalPosition GoalPos) {
    if (GoalPos == EGoalPosition::Random) {
        GoalPos = static_cast<EGoalPosition>(FMath::RandRange(0, 3));
    }
    
    FVector CenterPoint = GetActorLocation();
    float HalfOuterSize = OuterBoundarySize * 0.5f;
    
    // Get the opposite position
    EGoalPosition OppositePos = GetOppositePosition(GoalPos);
    
    UE_LOG(LogTemp, Display, TEXT("Goal position: %d, Opposite position: %d"), 
           static_cast<int>(GoalPos), static_cast<int>(OppositePos));
    
    // Calculate the drone location based on the opposite position
    // We can't use GetPositionLocation here as it might be using the same logic as for goal placement
    FVector DroneLocation = CenterPoint;
    FRotator FacingRotation(0.0f, 0.0f, 0.0f);
    
    // It's important to use OppositePos here, not GoalPos
    switch (OppositePos) {
        case EGoalPosition::Front:
            DroneLocation.X = CenterPoint.X + HalfOuterSize;
            FacingRotation.Yaw = 180.0f; // Face -X (inward)
            break;
        case EGoalPosition::Back:
            DroneLocation.X = CenterPoint.X - HalfOuterSize;
            FacingRotation.Yaw = 0.0f;   // Face +X (inward)
            break;
        case EGoalPosition::Left:
            DroneLocation.Y = CenterPoint.Y + HalfOuterSize;
            FacingRotation.Yaw = 270.0f; // Face -Y (inward)
            break;
        case EGoalPosition::Right:
            DroneLocation.Y = CenterPoint.Y - HalfOuterSize;
            FacingRotation.Yaw = 90.0f;  // Face +Y (inward)
            break;
        default:
            // Should never get here since we handle Random above
            break;
    }
    
    DroneLocation.Z = ObstacleSpawnHeight; // Set proper height
    
    // Additional logging for debugging
    UE_LOG(LogTemp, Display, TEXT("Moving drone to %s based on opposite position %d"), 
           *DroneLocation.ToString(), static_cast<int>(OppositePos));
    
    // Find all drones in the world and teleport them to this location
    TArray<AActor*> FoundDrones;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AQuadPawn::StaticClass(), FoundDrones);
    
    if (FoundDrones.Num() > 0) {
        // Get the first drone
        AQuadPawn* Drone = Cast<AQuadPawn>(FoundDrones[0]);
        if (Drone) {
            // Keep current pitch and roll, only change yaw
            FRotator CurrentRotation = Drone->GetActorRotation();
            FacingRotation.Pitch = CurrentRotation.Pitch;
            FacingRotation.Roll = CurrentRotation.Roll;
            
            // Teleport the drone to the exact location with rotation to face center
            Drone->SetActorLocationAndRotation(DroneLocation, FacingRotation);
            
            // Reset drone physics state if needed
            UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Drone->GetRootComponent());
            if (RootPrim && RootPrim->IsSimulatingPhysics()) {
                RootPrim->SetPhysicsLinearVelocity(FVector::ZeroVector);
                RootPrim->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            }
        }
    } else {
        UE_LOG(LogTemp, Warning, TEXT("No drones found in the world!"));
    }
}

FVector AObstacleManager::GetGoalPosition() const {
    if (SpawnedGoal) {
        return SpawnedGoal->GetActorLocation();
    }
    return FVector::ZeroVector;
}
