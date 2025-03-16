// ROS2Controller.cpp
#include "Controllers/ROS2Controller.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Materials/Material.h"
#include "GameFramework/Pawn.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonReader.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"

#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"

AROS2Controller::AROS2Controller()
    : bIsCapturing(false), ImagePublishTimer(0.0f)
{
    PrimaryActorTick.bCanEverTick = true;
    
    // Create ROS2 node component
    ROS2Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2Node"));
}

AROS2Controller::~AROS2Controller()
{
    // Cleanup will be handled in EndPlay
}


void AROS2Controller::Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FROS2Configuration& Config)
{
    DronePawn = InPawn;
    DroneController = InDroneController;
    Configuration = Config;
    
    // Set node name and namespace
    ROS2Node->Name = Configuration.NodeName;
    ROS2Node->Namespace = Configuration.Namespace;
    
    // Initialize ROS2 communication
    InitializeROS2();
    
    // Initialize image capture component
    InitializeImageCapture();
}

void AROS2Controller::BeginPlay()
{
    Super::BeginPlay();
    
    // Setup command subscriber
    FString commandTopic = Configuration.DroneID + "/" + Configuration.CommandTopicName;
    ROS2_CREATE_SUBSCRIBER(ROS2Node, this, commandTopic, UROS2StrMsg::StaticClass(), &AROS2Controller::HandleCommandMessage);

    // Setup state publisher (loop publisher that automatically publishes at the specified frequency)
    FString stateTopic = Configuration.DroneID + "/" + Configuration.StateTopicName;
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        ROS2Node,
        this,
        stateTopic,
        UROS2Publisher::StaticClass(),
        UROS2StrMsg::StaticClass(),
        Configuration.StatePublicationFrequencyHz,
        &AROS2Controller::UpdateStateMessage,
        UROS2QoS::SensorData,
        StatePublisher);
    
    // Setup image publisher (we'll manually publish in Tick)
    FString imageTopic = Configuration.DroneID + "/" + Configuration.ImageTopicName;
    ImagePublisher = ROS2Node->CreatePublisher(
        imageTopic,
        UROS2Publisher::StaticClass(),
        UROS2StrMsg::StaticClass(),
        UROS2QoS::SensorData);
}

void AROS2Controller::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Clean up ROS2 resources
    if (ROS2Node)
    {
        // Remove all publishers and subscribers before destroying the node
        if (StatePublisher)
        {
            StatePublisher = nullptr;
        }
        
        if (ImagePublisher)
        {
            ImagePublisher = nullptr;
        }
        
        // Let the component handle its own cleanup when it's destroyed
        ROS2Node->DestroyComponent();
        ROS2Node = nullptr;
    }
    
    Super::EndPlay(EndPlayReason);
}
void AROS2Controller::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Handle manual image publishing
    if (CaptureComponent && RenderTarget && ImagePublisher)
    {
        ImagePublishTimer += DeltaTime;
        if (ImagePublishTimer >= (1.0f / Configuration.ImagePublicationFrequencyHz))
        {
            PublishImage();
            ImagePublishTimer = 0.0f;
        }
    }
}

void AROS2Controller::SetDroneID(const FString& NewID)
{
    // Store the new ID
    Configuration.DroneID = NewID;
    
    // Reinitialize ROS2 with new ID
    if (ROS2Node)
    {
        // Clean up existing publishers
        if (StatePublisher)
        {
            StatePublisher = nullptr;
        }
        
        if (ImagePublisher)
        {
            ImagePublisher = nullptr;
        }
        
        // Destroy existing node component
        ROS2Node->DestroyComponent();
        
        // Create a new node component
        ROS2Node = NewObject<UROS2NodeComponent>(this, TEXT("ROS2Node"));
        ROS2Node->Name = Configuration.NodeName;
        ROS2Node->Namespace = Configuration.Namespace;
        
        // Reinitialize
        BeginPlay();
    }
}
void AROS2Controller::InitializeROS2()
{
    // Setup is done in BeginPlay
}

void AROS2Controller::InitializeImageCapture()
{
    // Create render target for camera
    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RenderTarget->InitCustomFormat(
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        PF_B8G8R8A8,  // Format that matches what we need for JPEG compression
        false);        // No need for HDR for streaming
    RenderTarget->UpdateResourceImmediate(true);
    
    // Create scene capture component
    CaptureComponent = NewObject<USceneCaptureComponent2D>(this);
    CaptureComponent->RegisterComponentWithWorld(GetWorld());
    CaptureComponent->AttachToComponent(DronePawn->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
    CaptureComponent->TextureTarget = RenderTarget;
    CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    CaptureComponent->bCaptureEveryFrame = false;  // We'll control when captures happen
    
    // Set up camera properties
    CaptureComponent->FOVAngle = 90.0f;  // Adjust FOV as needed
    
    // Position the camera on the drone
    // Note: Adjust these values based on your drone model
    CaptureComponent->SetRelativeLocation(FVector(50.0f, 0.0f, 0.0f));  // Forward-facing camera
    CaptureComponent->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f));
}

bool AROS2Controller::CaptureImage(TArray<FColor>& OutImageData)
{
    if (!CaptureComponent || !RenderTarget || bIsCapturing)
    {
        return false;
    }
    
    // Set flag to prevent concurrent captures
    bIsCapturing = true;
    
    // Capture the current view
    CaptureComponent->CaptureScene();
    
    // Read the pixels from the render target
    FRenderTarget* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    if (RenderTargetResource)
    {
        RenderTargetResource->ReadPixels(OutImageData);
        bIsCapturing = false;
        return true;
    }
    
    bIsCapturing = false;
    return false;
}

TArray<uint8> AROS2Controller::CompressImageData(const TArray<FColor>& ImageData)
{
    TArray<uint8> CompressedData;
    
    // Load the image wrapper module for JPEG compression
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
    
    if (ImageWrapper.IsValid() && ImageData.Num() > 0)
    {
        const int32 Width = Configuration.ImageResolution.X;
        const int32 Height = Configuration.ImageResolution.Y;
        
        // Compress the image data to JPEG
        ImageWrapper->SetRaw(
            ImageData.GetData(),
            ImageData.Num() * sizeof(FColor),
            Width,
            Height,
            ERGBFormat::BGRA,
            8  // Bits per channel
        );
        
        // Get the compressed data (quality 85 is a good balance)
        CompressedData = ImageWrapper->GetCompressed(85);
    }
    
    return CompressedData;
}

void AROS2Controller::PublishImage()
{
    if (!CaptureComponent || !RenderTarget || !ImagePublisher || bIsCapturing)
    {
        return;
    }
    
    TArray<FColor> ImageData;
    if (CaptureImage(ImageData))
    {
        // Compress the image to JPEG
        TArray<uint8> CompressedImageData = CompressImageData(ImageData);
        
        if (CompressedImageData.Num() > 0)
        {
            // Convert compressed data to base64
            FString Base64String = FBase64::Encode(CompressedImageData.GetData(), CompressedImageData.Num());
            
            // Create JSON with image info
            TSharedPtr<FJsonObject> ImageObject = MakeShareable(new FJsonObject);
            ImageObject->SetStringField("drone_id", Configuration.DroneID);
            ImageObject->SetNumberField("timestamp", FPlatformTime::Seconds());
            ImageObject->SetStringField("format", "jpeg");
            ImageObject->SetNumberField("width", Configuration.ImageResolution.X);
            ImageObject->SetNumberField("height", Configuration.ImageResolution.Y);
            ImageObject->SetStringField("data", Base64String);
            
            // Convert to string
            FString OutputString;
            TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
            FJsonSerializer::Serialize(ImageObject.ToSharedRef(), Writer);
            
            // Create and publish ROS message
            UROS2StrMsg* Message = Cast<UROS2StrMsg>(ImagePublisher->TopicMessage);
            if (Message)
            {
                FROSStr RosMsg;
                RosMsg.Data = OutputString;
                Message->SetMsg(RosMsg);
                ImagePublisher->Publish();
            }
        }
    }
}

void AROS2Controller::HandleCommandMessage(const UROS2GenericMsg* InMsg)
{
    const UROS2StrMsg* StringMsg = Cast<UROS2StrMsg>(InMsg);
    if (StringMsg)
    {
        FROSStr RosMsg;
        StringMsg->GetMsg(RosMsg);
        const FString& CommandStr = RosMsg.Data;
        
        // Parse command string
        if (CommandStr.StartsWith("RESET"))
        {
            HandleResetCommand(CommandStr);
        }
        else if (CommandStr.StartsWith("VEL"))
        {
            HandleVelocityCommand(CommandStr);
        }
        // Add other command types as needed
    }
}

void AROS2Controller::HandleResetCommand(const FString& CommandStr)
{
    if (DroneController)
    {
        // Parse the reset command for position
        // Example format: "RESET,0,0,1" (x,y,z)
        TArray<FString> Parts;
        CommandStr.ParseIntoArray(Parts, TEXT(","));
        
        if (Parts.Num() >= 4)
        {
            FVector ResetPosition(
                FCString::Atof(*Parts[1]),
                FCString::Atof(*Parts[2]),
                FCString::Atof(*Parts[3])
            );
            
            // Update the goal position
            CurrentGoalPosition = ResetPosition;
            
            // Reset the drone position
            // NOTE: You need to implement this method in your drone controller
            // DroneController->ResetDronePosition(ResetPosition);
            
            // For now, just log that we received the command
            UE_LOG(LogTemp, Log, TEXT("ROS2Controller: Received reset command to position (%f, %f, %f)"),
                  ResetPosition.X, ResetPosition.Y, ResetPosition.Z);
        }
    }
}

void AROS2Controller::HandleVelocityCommand(const FString& CommandStr)
{
    if (DroneController)
    {
        // Parse the velocity command
        // Example format: "VEL,0.5,-0.2,0.1,0.0" (linear_x, linear_y, linear_z, angular_z)
        TArray<FString> Parts;
        CommandStr.ParseIntoArray(Parts, TEXT(","));
        
        if (Parts.Num() >= 5)
        {
            FVector LinearVelocity(
                FCString::Atof(*Parts[1]),
                FCString::Atof(*Parts[2]),
                FCString::Atof(*Parts[3])
            );
            
            float AngularZ = FCString::Atof(*Parts[4]);
            
            // Set the velocity command
            // NOTE: You need to implement this method in your drone controller
            // DroneController->SetVelocityCommand(LinearVelocity, AngularZ);
            
            // For now, just log that we received the command
            UE_LOG(LogTemp, Log, TEXT("ROS2Controller: Received velocity command (%f, %f, %f, %f)"),
                  LinearVelocity.X, LinearVelocity.Y, LinearVelocity.Z, AngularZ);
        }
    }
}

void AROS2Controller::UpdateStateMessage(UROS2GenericMsg* InMessage)
{
    if (!DronePawn)
    {
        return;
    }
    
    UROS2StrMsg* StateMsg = Cast<UROS2StrMsg>(InMessage);
    if (StateMsg)
    {
        // Get current drone state
        FVector Position = DronePawn->GetActorLocation();
        FVector Velocity = DronePawn->GetVelocity();
        
        // Create JSON state data
        TSharedPtr<FJsonObject> StateObject = MakeShareable(new FJsonObject);
        StateObject->SetStringField("drone_id", Configuration.DroneID);
        StateObject->SetNumberField("timestamp", FPlatformTime::Seconds());
        
        // Position
        TSharedPtr<FJsonObject> PosObject = MakeShareable(new FJsonObject);
        PosObject->SetNumberField("x", Position.X);
        PosObject->SetNumberField("y", Position.Y);
        PosObject->SetNumberField("z", Position.Z);
        StateObject->SetObjectField("position", PosObject);
        
        // Linear velocity
        TSharedPtr<FJsonObject> VelObject = MakeShareable(new FJsonObject);
        VelObject->SetNumberField("x", Velocity.X);
        VelObject->SetNumberField("y", Velocity.Y);
        VelObject->SetNumberField("z", Velocity.Z);
        StateObject->SetObjectField("velocity", VelObject);
        
        // Convert to string
        FString OutputString;
        TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
        FJsonSerializer::Serialize(StateObject.ToSharedRef(), Writer);
        
        // Set the message
        FROSStr RosMsg;
        RosMsg.Data = OutputString;
        StateMsg->SetMsg(RosMsg);
    }
}