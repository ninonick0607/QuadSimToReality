#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NavigationComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UNavigationComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UNavigationComponent();

    void SetNavigationPlan(const TArray<FVector>& InWaypoints);
    FVector GetCurrentSetpoint() const;
    void UpdateNavigation(const FVector& CurrentPosition);
    void ResetNavigation();

    // ← new:
    UFUNCTION(BlueprintCallable, Category="Navigation")
    void SetCurrentDestination(const FVector& Destination);

    UFUNCTION(BlueprintCallable, Category="Navigation")
    void AddWaypoint(const FVector& Waypoint);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation")
    float AcceptableDistance = 100.0f;

private:
    TArray<FVector> Waypoints;
    int32 CurrentIndex;
};
