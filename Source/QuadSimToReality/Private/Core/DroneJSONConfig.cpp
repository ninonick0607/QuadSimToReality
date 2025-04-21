
// DroneJSONConfig.cpp
#include "Core/DroneJSONConfig.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonWriter.h"
#include "Serialization/JsonSerializer.h"

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
        (*FlightParams)->TryGetNumberField(TEXT("max_velocity_bound"), Config.FlightParams.MaxVelocityBound); 
        (*FlightParams)->TryGetNumberField(TEXT("max_velocity"), Config.FlightParams.MaxVelocity); 
        (*FlightParams)->TryGetNumberField(TEXT("max_angle"), Config.FlightParams.MaxAngle);
        (*FlightParams)->TryGetNumberField(TEXT("max_pid_output"), Config.FlightParams.MaxPIDOutput);
        (*FlightParams)->TryGetNumberField(TEXT("max_thrust"), Config.FlightParams.MaxThrust);
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

bool UDroneJSONConfig::SaveConfig()
 {
     // Build JSON object
     TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
 
     // Flight parameters
     TSharedPtr<FJsonObject> FlightParamsObj = MakeShared<FJsonObject>();
     FlightParamsObj->SetNumberField(TEXT("max_velocity_bound"),    Config.FlightParams.MaxVelocityBound);
     FlightParamsObj->SetNumberField(TEXT("max_velocity"),          Config.FlightParams.MaxVelocity);
     FlightParamsObj->SetNumberField(TEXT("max_angle"),             Config.FlightParams.MaxAngle);
     FlightParamsObj->SetNumberField(TEXT("max_pid_output"),        Config.FlightParams.MaxPIDOutput);
     FlightParamsObj->SetNumberField(TEXT("altitude_threshold"),    Config.FlightParams.AltitudeThreshold);
     FlightParamsObj->SetNumberField(TEXT("min_altitude_local"),    Config.FlightParams.MinAltitudeLocal);
     FlightParamsObj->SetNumberField(TEXT("acceptable_distance"),   Config.FlightParams.AcceptableDistance);
     JsonObject->SetObjectField(TEXT("flight_parameters"), FlightParamsObj);
 
     // Controller parameters
     TSharedPtr<FJsonObject> ControllerParamsObj = MakeShared<FJsonObject>();
     ControllerParamsObj->SetNumberField(TEXT("altitude_rate"),        Config.ControllerParams.AltitudeRate);
     ControllerParamsObj->SetNumberField(TEXT("yaw_rate"),             Config.ControllerParams.YawRate);
     ControllerParamsObj->SetNumberField(TEXT("min_velocity_for_yaw"), Config.ControllerParams.MinVelocityForYaw);
     JsonObject->SetObjectField(TEXT("controller"), ControllerParamsObj);
 
     // Obstacle parameters
     TSharedPtr<FJsonObject> ObstacleParamsObj = MakeShared<FJsonObject>();
     ObstacleParamsObj->SetNumberField(TEXT("outer_boundary"), Config.ObstacleParams.OuterBoundarySize);
     ObstacleParamsObj->SetNumberField(TEXT("inner_boundary"), Config.ObstacleParams.InnerBoundarySize);
     ObstacleParamsObj->SetNumberField(TEXT("spawn_height"),   Config.ObstacleParams.SpawnHeight);
     JsonObject->SetObjectField(TEXT("obstacle_parameters"), ObstacleParamsObj);
 
     // Serialize JSON to string
     FString OutputString;
     TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
     if (FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer))
     {
         // Save to file
         return FFileHelper::SaveStringToFile(OutputString, *GetConfigFilePath());
     }
     return false;
 }