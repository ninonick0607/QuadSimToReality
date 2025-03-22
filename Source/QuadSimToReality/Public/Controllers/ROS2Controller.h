#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "Msgs/ROS2Img.h"
#include "Msgs/ROS2Point.h"  
#include "Pawns/QuadPawn.h"  

#include "ROS2Controller.generated.h"

UCLASS()
class QUADSIMTOREALITY_API AROS2Controller : public AActor
{
    GENERATED_BODY()

public:
    AROS2Controller();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString NodeName = TEXT("quad_position_node"); 

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString Namespace = TEXT("quad_position");  

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    FString TopicName = TEXT("quad_position");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Image")
    FString ImageTopicName = TEXT("camera/image");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|Image")
    float ImagePublicationFrequencyHz = 15.f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2")
    float PublicationFrequencyHz = 30.f;

    UPROPERTY(EditAnywhere, Category = "ROS2")
    AQuadPawn* QuadPawn;  

protected:
    virtual void BeginPlay() override;

private:
    UFUNCTION()
    void UpdatePositionMessage(UROS2GenericMsg* InMessage);

    UPROPERTY()
    UROS2Publisher* ImagePublisher;

    UFUNCTION()
    void UpdateImageMessage(UROS2GenericMsg* InMessage);
    
    UPROPERTY()
    UROS2NodeComponent* Node;

    UPROPERTY()
    UROS2Publisher* PositionPublisher;

    int32 UpdateCount = 0;
};