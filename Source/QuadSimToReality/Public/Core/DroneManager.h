#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneManager.generated.h"

class AQuadPawn;
class AZMQController;
// Forward declaration for flight modes
enum class EFlightMode : uint8;

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
    
    // Register a quad-drone controller for global flight mode broadcasts
    void RegisterDroneController(class UQuadDroneController* Controller);

    // Toggle and query swarm mode
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    void SetSwarmMode(bool bEnable);
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    bool IsSwarmMode() const;

    // Get index of the given drone in the manager list
    UFUNCTION(BlueprintCallable, Category = "Swarm")
    int32 GetDroneIndex(AQuadPawn* Pawn) const;

    // Static accessor for the drone manager in the world
    static ADroneManager* Get(UWorld* World);

    // Delegate to broadcast global flight mode changes
    DECLARE_MULTICAST_DELEGATE_OneParam(FOnGlobalFlightModeChanged, EFlightMode /*NewMode*/);
    FOnGlobalFlightModeChanged OnGlobalFlightModeChanged;

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
    
private:
    // Whether swarm mode is enabled.
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category="Swarm", meta=(AllowPrivateAccess="true"))
    bool bSwarmMode = false;

	void OnActorSpawned(AActor* SpawnedActor);

    FDelegateHandle OnActorSpawnedHandle;
    // Last spawn location used for positioning new drones
    FVector LastSpawnLocation = FVector::ZeroVector;
};
