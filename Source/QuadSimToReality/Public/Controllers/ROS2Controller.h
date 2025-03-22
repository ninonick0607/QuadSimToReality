#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "Msgs/ROS2Img.h"
#include "Msgs/ROS2Point.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Pawns/QuadPawn.h"

#include "ROS2Controller.generated.h"

UCLASS()
class QUADSIMTOREALITY_API AROS2Controller : public AActor
{
    GENERATED_BODY()

public:
    AROS2Controller();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString NodeName = TEXT("quad_controller_node");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString Namespace = TEXT("quad");

    // Position Publishing
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString PositionTopicName = TEXT("position");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    float PositionFrequencyHz = 10.f;

    // Image Publishing
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Image")
    FString ImageTopicName = TEXT("camera/image");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Image")
    float ImageFrequencyHz = 15.f;

    UPROPERTY(EditAnywhere, Category = "ROS2|Image")
    FVector2D ImageResolution = FVector2D(128, 128);

    UPROPERTY(EditAnywhere, Category = "ROS2")
    AQuadPawn* QuadPawn;

    // Image Capture System
    UPROPERTY()
    TArray<UTextureRenderTarget2D*> RenderTargets;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Image")
    USceneCaptureComponent2D* SceneCapture;

    
protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void InitializeImageCapture();
    void CaptureImage();
    void ProcessCapturedImage(const TArray<FColor>& Pixels);

    UFUNCTION()
    void UpdatePositionMessage(UROS2GenericMsg* InMessage);

    UFUNCTION()
    void UpdateImageMessage(UROS2GenericMsg* InMessage);

    // ROS2 Components
    UPROPERTY()
    UROS2NodeComponent* Node;

    UPROPERTY()
    UROS2Publisher* PositionPublisher;

    UPROPERTY()
    UROS2Publisher* ImagePublisher;


    FTimerHandle CaptureTimerHandle;
    int32 CurrentRenderTargetIndex = 0;
    bool bIsProcessingImage = false;
    int32 UpdateCount = 0;
};