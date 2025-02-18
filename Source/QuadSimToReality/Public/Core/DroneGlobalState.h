#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"
#include "DroneGlobalState.generated.h"

class UQuadDroneController;
class AQuadPawn;
UCLASS()
class QUADSIMTOREALITY_API UDroneGlobalState : public UObject
{
    GENERATED_BODY()

public:
    /** Returns the global singleton instance (creates if not existing yet). */
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    static UDroneGlobalState* Get();

    // --- Existing usage from your old class ---
    
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void SetDesiredVelocity(const FVector& NewVelocity);

    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void BindController(UQuadDroneController* Controller);
    
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void UnbindController();

    UFUNCTION(BlueprintCallable, Category="GlobalState")
    const FVector& GetDesiredVelocity() const { return DesiredVelocity; }

    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void RegisterPawn(AQuadPawn* InPawn);

    TArray<TWeakObjectPtr<AQuadPawn>>& GetAllDrones() { return AllDrones; }

    /** Which drone's UI do we show in ImGui? */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GlobalState")
    int32 SelectedDroneIndex = 0;
    
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void DrawDroneManagerWindow(UWorld* InWorld, const FString& InDroneID);

private:
    /** Holds the single instance pointer. */
    static UDroneGlobalState* SingletonInstance;

private:
    /** Standard UObject-style constructor. */
    UDroneGlobalState();

    FVector DesiredVelocity;
    UQuadDroneController* BoundController;

    /** List of all spawned Pawns (kept as weak pointers). */
    TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

    int32 DroneCounter;
    TMap<AQuadPawn*, FString> PawnIDMap;
};