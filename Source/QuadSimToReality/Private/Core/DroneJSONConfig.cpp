
// DroneJSONConfig.cpp
#include "Core/DroneJSONConfig.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

UDroneJSONConfig* UDroneJSONConfig::Instance = nullptr;

UDroneJSONConfig::UDroneJSONConfig()
{
    LoadConfig();
}

UDroneJSONConfig& UDroneJSONConfig::Get()
{
    if (!Instance)
    {
        Instance = NewObject<UDroneJSONConfig>();
        Instance->AddToRoot(); 
    }
    return *Instance;
}

FString UDroneJSONConfig::GetConfigFilePath() const
{
    return FPaths::ProjectConfigDir() / TEXT("DroneConfig.json");
}

bool UDroneJSONConfig::LoadConfig()
{
    FString JsonString;
    if (!FFileHelper::LoadFileToString(JsonString, *GetConfigFilePath()))
    {
        return false;
    }

    TSharedPtr<FJsonObject> JsonObject;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);

    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        return false;
    }

    const TSharedPtr<FJsonObject>* FlightParams;
    if (JsonObject->TryGetObjectField(TEXT("flight_parameters"), FlightParams))
    {
        (*FlightParams)->TryGetNumberField(TEXT("max_velocity"), Config.FlightParams.MaxVelocity);
        (*FlightParams)->TryGetNumberField(TEXT("max_angle"), Config.FlightParams.MaxAngle);
        (*FlightParams)->TryGetNumberField(TEXT("max_pid_output"), Config.FlightParams.MaxPIDOutput);
        (*FlightParams)->TryGetNumberField(TEXT("altitude_threshold"), Config.FlightParams.AltitudeThreshold);
        (*FlightParams)->TryGetNumberField(TEXT("min_altitude_local"), Config.FlightParams.MinAltitudeLocal);
        (*FlightParams)->TryGetNumberField(TEXT("acceptable_distance"), Config.FlightParams.AcceptableDistance);
    }
    
    const TSharedPtr<FJsonObject>* ControllerParams;
    if (JsonObject->TryGetObjectField(TEXT("controller"), ControllerParams))
    {
        (*ControllerParams)->TryGetNumberField(TEXT("altitude_rate"), Config.ControllerParams.AltitudeRate);
        (*ControllerParams)->TryGetNumberField(TEXT("yaw_rate"), Config.ControllerParams.YawRate);
        (*ControllerParams)->TryGetNumberField(TEXT("min_velocity_for_yaw"), Config.ControllerParams.MinVelocityForYaw);
    }
    
    const TSharedPtr<FJsonObject>* ObstacleParams;
    if (JsonObject->TryGetObjectField(TEXT("obstacle_parameters"), ObstacleParams))
    {
        (*ObstacleParams)->TryGetNumberField(TEXT("outer_boundary"), Config.ObstacleParams.OuterBoundarySize);
        (*ObstacleParams)->TryGetNumberField(TEXT("inner_boundary"), Config.ObstacleParams.InnerBoundarySize);
        (*ObstacleParams)->TryGetNumberField(TEXT("spawn_height"), Config.ObstacleParams.SpawnHeight);
    }
    return true;
}

bool UDroneJSONConfig::ReloadConfig()
{
    return LoadConfig();
}