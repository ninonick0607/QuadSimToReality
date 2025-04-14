// Fill out your copyright notice in the Description page of Project Settings.

#include "Utility/NavigationComponent.h"
#include "Pawns/QuadPawn.h" // Include necessary headers
#include "Controllers/QuadDroneController.h"
#include "GameFramework/Actor.h" // For GetActorLocation
#include "Kismet/GameplayStatics.h" // Potentially useful later

// Sets default values for this component's properties
UNavigationComponent::UNavigationComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics; // Tick after physics update

	// Load defaults from config if needed, otherwise use hardcoded values
	// const auto& Config = UDroneJSONConfig::Get().Config;
	// AcceptanceRadius = Config.FlightParams.AcceptableDistance;
	// InitialClimbAltitude = Config.FlightParams.MinAltitudeLocal;

	CurrentActiveNavPlan = nullptr;
	CurrentWaypointIndex = -1;
	bAltitudeReached = false;
}

// Called when the game starts
void UNavigationComponent::BeginPlay()
{
	Super::BeginPlay();

	// Initial state setup can go here if not handled by Initialize or SetActivePlan
}

void UNavigationComponent::Initialize(AQuadPawn* InOwningPawn, UQuadDroneController* InController)
{
	OwningPawn = InOwningPawn;
	DroneController = InController;
	UE_LOG(LogTemp, Log, TEXT("NavigationComponent Initialized for Pawn: %s"), OwningPawn ? *OwningPawn->GetName() : TEXT("NULL"));
}


// Called every frame
void UNavigationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (IsNavigationActive())
	{
		UpdateWaypointProgress(DeltaTime);
	}
}

void UNavigationComponent::AddNavPlan(const FNavPlan& Plan)
{
	// Optional: Check if a plan with the same name already exists
	StoredNavPlans.Add(Plan);
	UE_LOG(LogTemp, Log, TEXT("Added Nav Plan: %s with %d waypoints"), *Plan.Name, Plan.Waypoints.Num());
}

bool UNavigationComponent::SetActivePlan(const FString& PlanName)
{
	for (int i = 0; i < StoredNavPlans.Num(); ++i)
	{
		if (StoredNavPlans[i].Name == PlanName)
		{
			CurrentActiveNavPlan = &StoredNavPlans[i];
			CurrentWaypointIndex = 0; // Start at the first waypoint
			bAltitudeReached = !bRequireInitialClimb; // Skip altitude check if not required
			UE_LOG(LogTemp, Log, TEXT("Set Active Nav Plan: %s"), *PlanName);

			CalculateInitialTarget(); // Set the first target

			return true;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Could not find Nav Plan: %s"), *PlanName);
	ResetNavigation(); // Clear state if plan not found
	return false;
}

void UNavigationComponent::CalculateInitialTarget()
{
	if (!CurrentActiveNavPlan || !OwningPawn || CurrentWaypointIndex < 0 || CurrentWaypointIndex >= CurrentActiveNavPlan->Waypoints.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("CalculateInitialTarget: Invalid state for calculation."));
		ResetNavigation();
		return;
	}

	if (!bAltitudeReached && bRequireInitialClimb)
	{
		FVector currentPos = OwningPawn->GetActorLocation();
		CurrentTargetWaypoint = FVector(currentPos.X, currentPos.Y, InitialClimbAltitude);
		UE_LOG(LogTemp, Log, TEXT("Navigation initial target set to climb altitude: Z=%.2f"), CurrentTargetWaypoint.Z);
	}
	else
	{
		// If climb not required or already done, target the first actual waypoint
		CurrentTargetWaypoint = CurrentActiveNavPlan->Waypoints[CurrentWaypointIndex];
		UE_LOG(LogTemp, Log, TEXT("Navigation initial target set to waypoint %d: %s"), CurrentWaypointIndex, *CurrentTargetWaypoint.ToString());
	}
}


void UNavigationComponent::UpdateWaypointProgress(float DeltaTime)
{
	if (!OwningPawn || !CurrentActiveNavPlan || CurrentWaypointIndex < 0)
	{
		// Should not happen if IsNavigationActive() check passed, but good for safety
		return;
	}

	FVector currentPosition = OwningPawn->GetActorLocation();
	float distanceToTarget = FVector::Dist(currentPosition, CurrentTargetWaypoint);

	if (distanceToTarget < AcceptanceRadius)
	{
		UE_LOG(LogTemp, Log, TEXT("Reached target: %s (Distance: %.2f)"), *CurrentTargetWaypoint.ToString(), distanceToTarget);

		if (!bAltitudeReached && bRequireInitialClimb)
		{
			// Just reached the initial climb altitude target
			bAltitudeReached = true;
			// Now set the target to the first *actual* waypoint
			CurrentWaypointIndex = 0; // Ensure we are at index 0
			if (CurrentWaypointIndex < CurrentActiveNavPlan->Waypoints.Num())
			{
				CurrentTargetWaypoint = CurrentActiveNavPlan->Waypoints[CurrentWaypointIndex];
				UE_LOG(LogTemp, Log, TEXT("Initial climb complete. Targeting waypoint %d: %s"), CurrentWaypointIndex, *CurrentTargetWaypoint.ToString());
				if (DroneController)
				{
					DroneController->ResetDroneIntegral(); // Reset PIDs after reaching climb altitude
				}
			}
			else
			{
				// Plan has no waypoints after climb? End navigation.
				UE_LOG(LogTemp, Warning, TEXT("Nav Plan %s has no waypoints after initial climb."), *CurrentActiveNavPlan->Name);
				ResetNavigation();
			}
		}
		else
		{
			// Reached a regular waypoint (or the first waypoint if climb wasn't required)
			CurrentWaypointIndex++;

			if (CurrentWaypointIndex < CurrentActiveNavPlan->Waypoints.Num())
			{
				// Set the next waypoint as the target
				CurrentTargetWaypoint = CurrentActiveNavPlan->Waypoints[CurrentWaypointIndex];
				UE_LOG(LogTemp, Log, TEXT("Proceeding to waypoint %d: %s"), CurrentWaypointIndex, *CurrentTargetWaypoint.ToString());
				if (DroneController)
				{
					DroneController->ResetDroneIntegral(); // Reset PIDs for the new waypoint
				}
			}
			else
			{
				// Reached the end of the plan
				UE_LOG(LogTemp, Log, TEXT("Navigation Plan '%s' completed."), *CurrentActiveNavPlan->Name);
				ResetNavigation(); // Clear active plan
			}
		}
	}
	// If distance is not within acceptance radius, do nothing - controller keeps flying towards CurrentTargetWaypoint
}


FVector UNavigationComponent::GetCurrentTargetWaypoint() const
{
	// Return the current target, or ZeroVector if navigation isn't active
	return IsNavigationActive() ? CurrentTargetWaypoint : FVector::ZeroVector;
}

bool UNavigationComponent::IsNavigationActive() const
{
	return CurrentActiveNavPlan != nullptr && CurrentWaypointIndex >= 0;
}

void UNavigationComponent::ResetNavigation()
{
	UE_LOG(LogTemp, Log, TEXT("Navigation Reset."));
	CurrentActiveNavPlan = nullptr;
	CurrentWaypointIndex = -1;
	bAltitudeReached = false;
	CurrentTargetWaypoint = FVector::ZeroVector; // Reset target
}

FString UNavigationComponent::GetActivePlanName() const
{
	return CurrentActiveNavPlan ? CurrentActiveNavPlan->Name : FString("");
}

int32 UNavigationComponent::GetCurrentWaypointIndex() const
{
	return CurrentWaypointIndex;
}