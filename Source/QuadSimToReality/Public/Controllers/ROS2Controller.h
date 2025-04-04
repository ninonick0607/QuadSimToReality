#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "ROS2Subscriber.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Msgs/ROS2Point.h"
#include "Msgs/ROS2Img.h"
#include "Msgs/ROS2Float64.h"
#include "Msgs/ROS2Twist.h"
#include "Msgs/ROS2Str.h"

class AObstacleManager;
class AQuadPawn;
class UQuadDroneController; 

#include "ROS2Controller.generated.h"

UCLASS()
class QUADSIMTOREALITY_API AROS2Controller : public AActor
{
	GENERATED_BODY()

public:
	AROS2Controller();

    // --- ROS2 Node Configuration ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
	FString NodeName = TEXT("quad_controller_node");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
	FString Namespace = TEXT("");

    // --- Publisher Topics & Config ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Position")
	FString PositionTopicName = TEXT("/drone/position");
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Position")
	float PositionFrequencyHz = 30.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Goal")
	FString PositionGoalTopicName = TEXT("/goal/position");
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Goal")
	float GoalFrequenzyHz = 1.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Image")
	FString ImageTopicName = TEXT("/camera/image");
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Image")
	float ImageFrequencyHz = 15.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Publishers|Image")
	FIntPoint ImageResolution = FIntPoint(128, 128);

    // --- Subscriber Topics ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Subscribers")
    FString ObstacleTopicName = TEXT("/obstacles");
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Subscribers") 
    FString CmdVelTopicName = TEXT("/cmd_vel");
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Subscribers") 
    FString ResetTopicName = TEXT("/reset");

    // --- Pawn & Manager References ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	AQuadPawn* QuadPawn;

    // --- Goal Altitude (Temporary) ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control|Goal") // <--- ADDED PROPERTY
    float TargetAltitude = 1000.0f; // Target Z in cm

    // --- Image Capture Component ---
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Image Capture")
    USceneCaptureComponent2D* SceneCapture;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // --- Helper Functions ---
	void InitializeImageCapture();
	void CaptureImage();
	void ProcessCapturedImage(const TArray<FColor>& Pixels);
	void SetupObstacleManager();

    // --- Publisher Update Callbacks ---
	UFUNCTION()
	void UpdatePositionMessage(UROS2GenericMsg* InMessage); // Still uses GenericMsg as base type
	UFUNCTION()
	void UpdateGoalPositionMessage(UROS2GenericMsg* InMessage);
	UFUNCTION()
	void UpdateImageMessage(UROS2GenericMsg* InMessage);
    // --- Subscriber Handler Callbacks ---
	UFUNCTION()
	void HandleObstacleMessage(const UROS2GenericMsg* InMsg); 
    UFUNCTION() 
    void HandleVelocityCommand(const UROS2GenericMsg* InMsg);
    UFUNCTION() 
    void HandleResetCommand(const UROS2GenericMsg* InMsg);

	// --- ROS2 Components (Internal) ---
	UPROPERTY()
	UROS2NodeComponent* Node;
	UPROPERTY()
	UROS2Publisher* PositionPublisher;
	UPROPERTY()
	UROS2Publisher* ImagePublisher;
	UPROPERTY()
	UROS2Publisher* GoalPosition;
	UPROPERTY()
	UROS2Subscriber* ObstacleSubscriber;
    UPROPERTY()
    UROS2Subscriber* CmdVelSubscriber;
    UPROPERTY()
    UROS2Subscriber* ResetSubscriber;

    // --- Internal State ---
	UPROPERTY()
	AObstacleManager* ObstacleManagerInstance;
	UPROPERTY()
	TArray<UTextureRenderTarget2D*> RenderTargets;
	FTimerHandle CaptureTimerHandle;
	int32 CurrentRenderTargetIndex = 0;
	bool bIsProcessingImage = false;
};