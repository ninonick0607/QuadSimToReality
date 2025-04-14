// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NavigationComponent.generated.h"

class AQuadPawn; // Forward declaration
class UQuadDroneController; // Forward declaration

// Structure to hold a navigation plan (can be expanded later)
// Making it a USTRUCT allows easier reflection and potential Blueprint use later
USTRUCT(BlueprintType)
struct FNavPlan
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation")
	TArray<FVector> Waypoints;

	FNavPlan() : Name("DefaultPlan") {}
};


UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UNavigationComponent : public UActorComponent // Replace YOURPROJECTNAME_API
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UNavigationComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Initializes the component, linking it to the owning pawn and its controller. */
	void Initialize(AQuadPawn* InOwningPawn, UQuadDroneController* InController);

	/** Adds a navigation plan to the stored list. */
	void AddNavPlan(const FNavPlan& Plan);

	/** Sets the active navigation plan by name. Resets progress. */
	bool SetActivePlan(const FString& PlanName);

	/** Gets the current target waypoint the drone should fly towards. */
	FVector GetCurrentTargetWaypoint() const;

	/** Checks if a navigation plan is currently active and running. */
	bool IsNavigationActive() const;

	/** Resets the navigation state, clearing the active plan and progress. */
	void ResetNavigation();

	/** Gets the name of the currently active plan, or empty string if none. */
	FString GetActivePlanName() const;

	/** Gets the current waypoint index. Returns -1 if no plan active. */
	int32 GetCurrentWaypointIndex() const;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	/** Updates the target waypoint based on the current plan and drone position. */
	void UpdateWaypointProgress(float DeltaTime);

	/** Calculates the initial target when a plan starts (handles initial climb). */
	void CalculateInitialTarget();

	// --- Configuration ---

	/** The distance threshold to consider a waypoint reached (in cm). */
	UPROPERTY(EditAnywhere, Category = "Navigation")
	float AcceptanceRadius = 50.0f; // Default value, same as old acceptableDistance

	/** The minimum altitude the drone should reach before starting the waypoint sequence (in cm). */
	UPROPERTY(EditAnywhere, Category = "Navigation")
	float InitialClimbAltitude = 1000.0f; // Default value, same as old minAltitudeLocal

	/** If true, the drone must reach InitialClimbAltitude before proceeding to the first waypoint. */
	UPROPERTY(EditAnywhere, Category = "Navigation")
	bool bRequireInitialClimb = true; // Set to false if not needed


	// --- State ---

	/** List of all available navigation plans. */
	UPROPERTY() // Keep UPROPERTY to prevent garbage collection if plans are added dynamically
		TArray<FNavPlan> StoredNavPlans;

	/** Pointer to the currently active navigation plan within StoredNavPlans. */
	FNavPlan* CurrentActiveNavPlan = nullptr;

	/** Index of the current target waypoint within the CurrentActiveNavPlan. */
	int32 CurrentWaypointIndex = -1;

	/** The specific world-space location the drone controller should currently target. */
	FVector CurrentTargetWaypoint = FVector::ZeroVector;

	/** Flag indicating if the initial climb altitude has been reached. */
	bool bAltitudeReached = false;

	/** Pointer to the owning pawn (needed for GetActorLocation). */
	UPROPERTY()
	TObjectPtr<AQuadPawn> OwningPawn = nullptr;

	/** Pointer to the drone's controller (needed to reset integrals). */
	UPROPERTY()
	TObjectPtr<UQuadDroneController> DroneController = nullptr;
};