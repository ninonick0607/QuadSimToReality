#include "Utility/NavigationComponent.h"

UNavigationComponent::UNavigationComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    CurrentIndex = 0;
}

void UNavigationComponent::SetNavigationPlan(const TArray<FVector>& InWaypoints)
{
    Waypoints = InWaypoints;
    CurrentIndex = 0;
}

FVector UNavigationComponent::GetCurrentSetpoint() const
{
    if (Waypoints.IsValidIndex(CurrentIndex))
    {
        return Waypoints[CurrentIndex];
    }
    return FVector::ZeroVector;
}

void UNavigationComponent::UpdateNavigation(const FVector& CurrentPosition)
{
    if (!Waypoints.IsValidIndex(CurrentIndex))
        return;

    if (FVector::Dist(CurrentPosition, Waypoints[CurrentIndex]) < AcceptableDistance)
    {
        CurrentIndex++;
        if (CurrentIndex >= Waypoints.Num())
        {
            CurrentIndex = 0; // Or handle plan completion as needed.
        }
    }
}

void UNavigationComponent::ResetNavigation()
{
    CurrentIndex = 0;
}
