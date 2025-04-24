// DroneJSONConfig.h
#pragma once

#include "CoreMinimal.h"
#include "DroneJSONConfig.generated.h"

USTRUCT()
struct FDroneConfigData
{
	GENERATED_BODY()

	struct FFlightParameters {
		float MaxVelocityBound;
		float MaxVelocity;
		float MaxAngle;
		float MaxPIDOutput;
		float MaxThrust;
		float AltitudeThreshold;
		float MinAltitudeLocal;
		float AcceptableDistance;
	} FlightParams;

	struct FControllerParameters {
		float AltitudeRate;
		float YawRate;
		float MinVelocityForYaw;
	} ControllerParams;

	struct FObstacleParameters
	{
		float InnerBoundarySize;
		float OuterBoundarySize;
		float SpawnHeight;
	} ObstacleParams;
};

UCLASS()
class QUADSIMTOREALITY_API UDroneJSONConfig : public UObject
{
	GENERATED_BODY()

public:
	UDroneJSONConfig();
    
	static UDroneJSONConfig& Get();
	bool LoadConfig();
	bool ReloadConfig();
	UFUNCTION(BlueprintCallable, Category = "Drone Config")
	bool SaveConfig();
	FDroneConfigData Config;

private:
	static UDroneJSONConfig* Instance;
	FString GetConfigFilePath() const;
};