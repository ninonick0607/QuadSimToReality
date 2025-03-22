#include "Controllers/ROS2Controller.h"
#include "ROS2Node.h"

AROS2Controller::AROS2Controller()
{
    PrimaryActorTick.bCanEverTick = false;

    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    Node->Name = NodeName;
    Node->Namespace = Namespace;
}

// CORRECTED ROS2Controller.cpp
void AROS2Controller::BeginPlay()
{
    Super::BeginPlay();

    if(!QuadPawn)
    {
        UE_LOG(LogTemp, Error, TEXT("QuadPawn reference not set!"));
        return;
    }

    Node->Init();

    // Position publisher
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node,
        this,
        TopicName,
        UROS2Publisher::StaticClass(),
        UROS2PointMsg::StaticClass(),
        PublicationFrequencyHz,
        &AROS2Controller::UpdatePositionMessage,
        UROS2QoS::Services,  // Predefined reliable QoS
        PositionPublisher
    );

    // Image publisher
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node,
        this,
        ImageTopicName,
        UROS2Publisher::StaticClass(),
        UROS2ImgMsg::StaticClass(),
        ImagePublicationFrequencyHz,
        &AROS2Controller::UpdateImageMessage,
        UROS2QoS::SensorData,  // Predefined sensor QoS
        ImagePublisher
    );
}
void AROS2Controller::UpdatePositionMessage(UROS2GenericMsg* InMessage)
{
    if(!QuadPawn) return;

    FROSPoint positionMsg;
    const FVector WorldPosition = QuadPawn->GetActorLocation();
    
    positionMsg.X = WorldPosition.X;
    positionMsg.Y = WorldPosition.Y;
    positionMsg.Z = WorldPosition.Z;

    CastChecked<UROS2PointMsg>(InMessage)->SetMsg(positionMsg);
    
    if(++UpdateCount % 60 == 0)  // Log once per second at 60Hz
    {
        UE_LOG(LogTemp, Display, TEXT("Position Update: X=%.2f, Y=%.2f, Z=%.2f"), 
            WorldPosition.X, WorldPosition.Y, WorldPosition.Z);
    }
}

void AROS2Controller::UpdateImageMessage(UROS2GenericMsg* InMessage)
{
    if (!QuadPawn) return;

    TArray<uint8> ImageData;
    QuadPawn->CaptureCameraImage(ImageData);

    // Add image capture logging
    if (ImageData.Num() > 0)
    {
        UE_LOG(LogTemp, Display, TEXT("Captured image: %d bytes"), ImageData.Num());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to capture image data!"));
        return;
    }

    FROSImg ImageMsg;
    ImageMsg.Height = 128;
    ImageMsg.Width = 128;
    ImageMsg.Encoding = "rgb8";
    ImageMsg.Step = 128 * 3;
    ImageMsg.Data = ImageData;

    CastChecked<UROS2ImgMsg>(InMessage)->SetMsg(ImageMsg);
    UE_LOG(LogTemp, Verbose, TEXT("Published image message"));
}