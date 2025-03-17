// ROS2Controller.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// ROS2 high-level components
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"

// Message types (UE-side wrappers)
#include "Msgs/ROS2Str.h"

// Forward declarations
class AQuadPawn;
class UQuadDroneController;
class UTextureRenderTarget2D;
class USceneCaptureComponent2D;

#include "ROS2Controller.generated.h"

USTRUCT(BlueprintType)
struct FROS2Configuration
{
    GENERATED_BODY()

    // Topic names
    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString StateTopicName = "drone_state";

    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString CommandTopicName = "drone_cmd";

    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString ImageTopicName = "drone_camera";

    // Node and QoS settings
    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString NodeName = "ue_drone_node";

    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString Namespace = "sim";

    UPROPERTY(EditAnywhere, Category = "ROS2")
    FString DroneID = "drone1";

    // Publication frequencies
    UPROPERTY(EditAnywhere, Category = "ROS2")
    float StatePublicationFrequencyHz = 30.0f;  // 30 Hz state updates

    // Image capture settings
    UPROPERTY(EditAnywhere, Category = "Image Capture")
    FIntPoint ImageResolution = FIntPoint(128, 128);

    UPROPERTY(EditAnywhere, Category = "Image Capture")
    float ImagePublicationFrequencyHz = 10.0f;  // 10 Hz image updates
};

/**
 * ROS2Controller - Implements ROS2 communication for the drone
 * using the high-level rclUE components
 */
UCLASS(Blueprintable)
class QUADSIMTOREALITY_API AROS2Controller : public AActor
{
    GENERATED_BODY()
    
public:
    AROS2Controller();
    virtual ~AROS2Controller();

    // Initialization method
    UFUNCTION(BlueprintCallable, Category = "ROS2")
    void Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FROS2Configuration& Config);

    // Configuration getter
    UFUNCTION(BlueprintCallable, Category = "ROS2")
    const FROS2Configuration& GetConfiguration() const { return Configuration; }

    // Get current goal position
    UFUNCTION(BlueprintCallable, Category = "ROS2")
    FVector GetCurrentGoalPosition() const { return CurrentGoalPosition; }

    // Set drone ID
    UFUNCTION(BlueprintCallable, Category = "ROS2")
    void SetDroneID(const FString& NewID);

    // Target pawn this controller is responsible for
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS2")
    AQuadPawn* TargetPawn;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

    // Message callbacks
    UFUNCTION()
    void HandleCommandMessage(const UROS2GenericMsg* InMsg);
    
    // State update callback for loop publisher
    UFUNCTION()
    void UpdateStateMessage(UROS2GenericMsg* InMessage);
    
    // Manual image publication method (called from Tick)
    void PublishImage();

private:
    // ROS2 initialization
    void InitializeImageCapture();
    
    // Command handlers
    void HandleResetCommand(const FString& CommandStr);
    void HandleVelocityCommand(const FString& CommandStr);
    
    // Image utilities
    bool CaptureImage(TArray<FColor>& OutImageData);
    TArray<uint8> CompressImageData(const TArray<FColor>& ImageData);

    // ROS2 node component
    UPROPERTY()
    UROS2NodeComponent* ROS2Node;
    
    // Publishers
    UPROPERTY()
    UROS2Publisher* StatePublisher;
    
    UPROPERTY()
    UROS2Publisher* ImagePublisher;
    
    // Configuration and state
    UPROPERTY(EditAnywhere, Category = "ROS2")
    FROS2Configuration Configuration;
    
    // References to the drone and controller
    UPROPERTY()
    AQuadPawn* DronePawn;

    UPROPERTY()
    UQuadDroneController* DroneController;

    // Components for camera/image capture
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    // Current goal position
    FVector CurrentGoalPosition;
    
    // Flag to track if we're currently capturing an image
    bool bIsCapturing;
    
    // Timer for manual image publishing
    float ImagePublishTimer;
};