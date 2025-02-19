#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"  // Needed for actor lookup
#include "DroneManager.generated.h"

class AQuadPawn;

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

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	int32 SelectedDroneIndex;
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone Manager")
		TSubclassOf<AQuadPawn> QuadPawnClass;

	UPROPERTY(VisibleAnywhere, Category = "Drone Manager")
	TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

	void OnActorSpawned(AActor* SpawnedActor);

	FDelegateHandle OnActorSpawnedHandle;
	
};
