#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"   // Common base for UObject in a minimal module
#include "UObject/WeakObjectPtr.h"
#include "DroneGlobalState.generated.h"

class UQuadDroneController;
class AQuadPawn;

UCLASS()
class QUADSIMTOREALITY_API UDroneGlobalState : public UObject
{
    GENERATED_BODY()

public:
    // (Existing public functions...)
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    static UDroneGlobalState* Get();

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

    // Note: We don’t expose GetAllDrones to Blueprint.
    TArray<TWeakObjectPtr<AQuadPawn>>& GetAllDrones() { return AllDrones; }

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GlobalState")
    int32 SelectedDroneIndex = 0;
    
    UFUNCTION(BlueprintCallable, Category="GlobalState")
    void DrawDroneManagerWindow(UWorld* InWorld, const FString& InDroneID);

private:
    static UDroneGlobalState* SingletonInstance;
    UDroneGlobalState();

    FVector DesiredVelocity;
    UQuadDroneController* BoundController;

    TArray<TWeakObjectPtr<AQuadPawn>> AllDrones;

    int32 DroneCounter;
    TMap<AQuadPawn*, FString> PawnIDMap;
};

