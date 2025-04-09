// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ObstacleManager.generated.h"

UENUM(BlueprintType)
enum class EGoalPosition : uint8
{
    Front,
    Back,
    Left,
    Right,
    Random
};

UCLASS(Blueprintable)
class QUADSIMTOREALITY_API AObstacleManager : public AActor {
    GENERATED_BODY()

public:
    AObstacleManager();
    
    // Boundary properties - loaded from JSON
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boundary")
    float InnerBoundarySize;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boundary")
    float OuterBoundarySize;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boundary")
    float ObstacleSpawnHeight;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
    TSubclassOf<AActor> ObstacleClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
    TSubclassOf<AActor> GoalClass;
   
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* VisualMarker;
   
    // Main function to create obstacles and goal - called by ZMQ controller
    UFUNCTION(BlueprintCallable, Category = "Obstacles")
    void CreateObstacles(int32 NumObstacles, EGoalPosition GoalPos = EGoalPosition::Random);
   
    // Clear all obstacles and goal
    UFUNCTION(BlueprintCallable, Category = "Obstacles")
    void ClearObstacles();

    UFUNCTION(BlueprintCallable, Category = "Obstacles")
    void MoveDroneToOppositeOfGoal(EGoalPosition GoalPos);

    //UFUNCTION(BlueprintCallable, Category = "Obstacles")
    //FVector GetObstaclePosition() const { return GoalPosition; }
   
    UFUNCTION(BlueprintCallable, Category = "Goal")
    FVector GetGoalPosition() const;

protected:
    virtual void BeginPlay() override;
   
    // Draw debug visualization with thicker lines
    UFUNCTION(BlueprintCallable, Category = "Debug")
    void VisualizeSpawnBoundaries(bool bPersistentLines = false);
   
private:
    // Randomly place a single obstacle within inner boundary
    AActor* SpawnObstacle();
   
    // Place goal at specified boundary face
    AActor* SpawnGoal(EGoalPosition Position);
   
    // Get random point within inner boundary
    FVector GetRandomInnerPoint();

    EGoalPosition GetOppositePosition(EGoalPosition Position);
    FVector GetPositionLocation(EGoalPosition Position);
   
    // Store spawned actors for easy cleanup
    UPROPERTY()
    TArray<AActor*> SpawnedObstacles;
   
    UPROPERTY()
    AActor* SpawnedGoal;

    FVector GoalPosition;

};