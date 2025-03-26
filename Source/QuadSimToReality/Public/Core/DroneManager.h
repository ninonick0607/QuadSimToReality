#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneManager.generated.h"

class AQuadPawn;
class AROS2Controller;

UCLASS()
class QUADSIMTOREALITY_API ADroneManager : public AActor
{
	GENERATED_BODY()

public:
	ADroneManager();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
    
	UFUNCTION(BlueprintCallable, Category = "Drone Manager")
	AQuadPawn* SpawnDrone(const FVector& SpawnLocation, const FRotator& SpawnRotation);

	UFUNCTION(BlueprintCallable, Category = "Drone Manager")
	TArray<AQuadPawn*> GetDroneList() const;

	// Function for ROS2Controllers to register themselves.
	UFUNCTION(BlueprintCallable, Category = "Drone Manager")
	void RegisterROS2Controller(AROS2Controller* Controller);

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	int32 SelectedDroneIndex;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
	TSubclassOf<AQuadPawn> QuadPawnClass;

	// The blueprint class for ROS2Controller.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
	TSubclassOf<AROS2Controller> ROS2ControllerClass;

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

	// Array to keep track of all spawned ROS2Controllers.
	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AROS2Controller>> AllROS2Controllers;

	void OnActorSpawned(AActor* SpawnedActor);

	FDelegateHandle OnActorSpawnedHandle;
};