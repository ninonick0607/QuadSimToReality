#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneManager.generated.h"

class AQuadPawn;
class AZMQController;

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

	// Function for ZMQControllers to register themselves.
	UFUNCTION(BlueprintCallable, Category = "Drone Manager")
	void RegisterZMQController(AZMQController* Controller);

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	int32 SelectedDroneIndex;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
	TSubclassOf<AQuadPawn> QuadPawnClass;

	// New: The blueprint class for ZMQController.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
	TSubclassOf<AZMQController> ZMQControllerClass;

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

	// New: Array to keep track of all spawned ZMQControllers.
	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AZMQController>> AllZMQControllers;

	void OnActorSpawned(AActor* SpawnedActor);

	FDelegateHandle OnActorSpawnedHandle;
};
