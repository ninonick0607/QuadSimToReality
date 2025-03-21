// Fill out your copyright notice in the Description page of Project Settings.

#include "Core/DroneJSONConfig.h"
#include "Utility/ObstacleManager.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Pawns/QuadPawn.h"


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
        FColor(0, 0, 255, 128),  // More visible blue
        bPersistentLines,
        Duration,
        0,
        18.0f  // Thicker lines
    );
    
    // Draw the inner boundary - red with increased alpha and thickness
    DrawDebugBox(
        GetWorld(),
        CenterPoint,
        InnerExtent,
        FColor(255, 0, 0, 128),  // More visible red
        bPersistentLines,
        Duration,
        0,
        18.0f  // Thicker lines
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
    
    // More detailed logging
    UE_LOG(LogTemp, Display, TEXT("Trying to spawn obstacle at %s (Inner Boundary=%f)"), 
           *SpawnLocation.ToString(), InnerBoundarySize);
    
    // Random rotation around Z axis only
    FRotator SpawnRotation(0.0f, FMath::RandRange(0.0f, 360.0f), 0.0f);
    
    // Spawn parameters
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    
    // Spawn the actor using the class instead of template
    AActor* Obstacle = GetWorld()->SpawnActor<AActor>(ObstacleClass, 
                                                     SpawnLocation, 
                                                     SpawnRotation, 
                                                     SpawnParams);
    
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
    // REMOVE THIS LINE: SpawnParams.Template = GoalClass; 
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
    
    // Spawn goal
    SpawnedGoal = SpawnGoal(GoalPos);
    
    // Move drone to opposite of goal
    MoveDroneToOppositeOfGoal(GoalPos);
    
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
    // If random was selected, pick one of the four positions
    if (GoalPos == EGoalPosition::Random) {
        GoalPos = static_cast<EGoalPosition>(FMath::RandRange(0, 3));
    }
    
    FVector CenterPoint = GetActorLocation();
    float HalfOuterSize = OuterBoundarySize * 0.5f;
    
    // Get the opposite position
    EGoalPosition OppositePos = GetOppositePosition(GoalPos);
    
    // Calculate the drone location directly on the opposite face
    FVector DroneLocation = CenterPoint;
    FRotator FacingRotation;
    
    // Explicitly set the position based on which face we need
    switch (OppositePos) {
        case EGoalPosition::Front:
            DroneLocation.X = CenterPoint.X + HalfOuterSize;
            FacingRotation = FRotator(0.0f, 180.0f, 0.0f); // Face inward
            break;
            
        case EGoalPosition::Back:
            DroneLocation.X = CenterPoint.X - HalfOuterSize;
            FacingRotation = FRotator(0.0f, 0.0f, 0.0f); // Face inward
            break;
            
        case EGoalPosition::Left:
            DroneLocation.Y = CenterPoint.Y + HalfOuterSize;
            FacingRotation = FRotator(0.0f, 270.0f, 0.0f); // Face inward
            break;
            
        case EGoalPosition::Right:
            DroneLocation.Y = CenterPoint.Y - HalfOuterSize;
            FacingRotation = FRotator(0.0f, 90.0f, 0.0f); // Face inward
            break;
            
        default:
            break;
    }
    
    DroneLocation.Z = ObstacleSpawnHeight; // Set proper height
    
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
            
            UE_LOG(LogTemp, Display, TEXT("Moved drone to %s (opposite of %s): Location=%s"),
                   *UEnum::GetValueAsString(TEXT("EGoalPosition"), OppositePos),
                   *UEnum::GetValueAsString(TEXT("EGoalPosition"), GoalPos),
                   *DroneLocation.ToString());
        }
    } else {
        UE_LOG(LogTemp, Warning, TEXT("No drones found in the world!"));
    }
}