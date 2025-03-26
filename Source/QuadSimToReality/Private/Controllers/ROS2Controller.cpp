#include "Controllers/ROS2Controller.h"
#include "ROS2Node.h"
#include "Kismet/GameplayStatics.h"
#include "Async/Async.h"
#include "Msgs/ROS2Float32.h"
#include "Msgs/ROS2Float64.h"
#include "Msgs/ROS2Str.h"

AROS2Controller::AROS2Controller()
{
    PrimaryActorTick.bCanEverTick = false;

    // Initialize ROS2 node component
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    
    // Initialize SceneCapture component
    SceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
    SceneCapture->SetupAttachment(RootComponent);  // Attach to root temporarily

    // Initialize other components
    RenderTargets.SetNum(2);
}

void AROS2Controller::BeginPlay()
{
    Super::BeginPlay();

    if(!QuadPawn)
    {
        UE_LOG(LogTemp, Error, TEXT("QuadPawn reference not set!"));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("ROS2Controller: Initializing with namespace '%s'"), *Namespace);
    
    // Initialize ROS2 node
    Node->Name = NodeName;
    Node->Namespace = Namespace;
    Node->Init();
    
    // Log the fully qualified topic names
    UE_LOG(LogTemp, Warning, TEXT("Position topic: %s"), *PositionTopicName);
    UE_LOG(LogTemp, Warning, TEXT("Image topic: %s"), *ImageTopicName);
    UE_LOG(LogTemp, Warning, TEXT("Obstacle topic: %s"), *ObstacleTopicName);

    // Setup position publisher
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node,
        this,
        PositionTopicName,
        UROS2Publisher::StaticClass(),
        UROS2PointMsg::StaticClass(),
        PositionFrequencyHz,
        &AROS2Controller::UpdatePositionMessage,
        UROS2QoS::Services,
        PositionPublisher
    );

    // Setup image publisher
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node,
        this,
        ImageTopicName,
        UROS2Publisher::StaticClass(),
        UROS2ImgMsg::StaticClass(),
        ImageFrequencyHz,
        &AROS2Controller::UpdateImageMessage,
        UROS2QoS::SensorData,
        ImagePublisher
    );

    SetupObstacleManager();
    
    UE_LOG(LogTemp, Warning, TEXT("Setting up obstacle subscriber on topic: %s"), *ObstacleTopicName);
    
    // Create obstacle subscriber with standard macro
    ROS2_CREATE_SUBSCRIBER(
        Node,
        this,
        ObstacleTopicName,
        UROS2Float64Msg::StaticClass(),
        &AROS2Controller::HandleObstacleMessage
    );
    
    UE_LOG(LogTemp, Warning, TEXT("Obstacle subscriber created successfully"));

    // Initialize image capture system
    InitializeImageCapture();

    // Start image capture timer
    if (ImageFrequencyHz > 0)
    {
        GetWorld()->GetTimerManager().SetTimer(
            CaptureTimerHandle,
            this,
            &AROS2Controller::CaptureImage,
            1.0f / ImageFrequencyHz,
            true
        );
    }
}
void AROS2Controller::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorld()->GetTimerManager().ClearTimer(CaptureTimerHandle);
    Super::EndPlay(EndPlayReason);
}

void AROS2Controller::InitializeImageCapture()
{
    if (!QuadPawn || !QuadPawn->CameraFPV || !SceneCapture)
    {
        UE_LOG(LogTemp, Error, TEXT("Missing required components for image capture!"));
        return;
    }

    // Only attach if not already attached
    if (!SceneCapture->IsAttachedTo(QuadPawn->CameraFPV))
    {
        SceneCapture->AttachToComponent(QuadPawn->CameraFPV, 
            FAttachmentTransformRules::SnapToTargetIncludingScale);
    }
    SceneCapture->RegisterComponent();

    // Configure render targets with BGR format
    for (int32 i = 0; i < 2; ++i)
    {
        RenderTargets[i] = NewObject<UTextureRenderTarget2D>(this);
        RenderTargets[i]->InitCustomFormat(
            ImageResolution.X,
            ImageResolution.Y,
            PF_B8G8R8A8,  // Changed to BGR format
            true  // sRGB
        );
        RenderTargets[i]->TargetGamma = 2.2f;
        RenderTargets[i]->UpdateResourceImmediate(true);
    }

    // Configure scene capture settings
    SceneCapture->FOVAngle = QuadPawn->CameraFPV->FieldOfView;
    SceneCapture->PostProcessSettings = QuadPawn->CameraFPV->PostProcessSettings;
    SceneCapture->ShowFlags.SetTonemapper(false);
    SceneCapture->ShowFlags.SetColorGrading(false);
    SceneCapture->ShowFlags.SetEyeAdaptation(false);
    SceneCapture->CaptureSource = SCS_FinalColorLDR;
    SceneCapture->bCaptureEveryFrame = false;
}

void AROS2Controller::CaptureImage()
{
    if (!SceneCapture || bIsProcessingImage) return;

    SceneCapture->SetWorldLocationAndRotation(
        QuadPawn->CameraFPV->GetComponentLocation(),
        QuadPawn->CameraFPV->GetComponentRotation()
    );
    
    CurrentRenderTargetIndex = (CurrentRenderTargetIndex + 1) % 2;
    UTextureRenderTarget2D* CurrentTarget = RenderTargets[CurrentRenderTargetIndex];
    
    // Update scene capture
    SceneCapture->TextureTarget = CurrentTarget;
    SceneCapture->CaptureScene();

    // Process capture on render thread
    FTextureRenderTargetResource* RTResource = CurrentTarget->GameThread_GetRenderTargetResource();
    if (!RTResource) return;

    bIsProcessingImage = true;
    
    ENQUEUE_RENDER_COMMAND(CaptureImageCommand)(
        [this, RTResource](FRHICommandListImmediate& RHICmdList)
        {
            TArray<FColor> Pixels;
            RHICmdList.ReadSurfaceData(
                RTResource->GetRenderTargetTexture(),
                FIntRect(0, 0, ImageResolution.X, ImageResolution.Y),
                Pixels,
                FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
            );

            AsyncTask(ENamedThreads::GameThread, [this, Pixels]()
            {
                ProcessCapturedImage(Pixels);
            });
        }
    );

    
}

void AROS2Controller::ProcessCapturedImage(const TArray<FColor>& Pixels)
{
    if (Pixels.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to capture image data"));
        bIsProcessingImage = false;
        return;
    }

    // Convert to ROS image message
    FROSImg ImageMsg;
    ImageMsg.Height = ImageResolution.Y;
    ImageMsg.Width = ImageResolution.X;
    ImageMsg.Encoding = "bgr8";  // Change to bgr8 since we're sending in BGR order
    ImageMsg.Step = ImageResolution.X * 3;
    ImageMsg.Data.Reserve(Pixels.Num() * 3);

    for (const FColor& Pixel : Pixels)
    {
        // Store in BGR order to match the expected format
        ImageMsg.Data.Add(Pixel.B);  
        ImageMsg.Data.Add(Pixel.G);  
        ImageMsg.Data.Add(Pixel.R);  
    }

    if (IsValid(ImagePublisher) && IsValid(ImagePublisher->TopicMessage))
    {
        if (UROS2ImgMsg* Msg = Cast<UROS2ImgMsg>(ImagePublisher->TopicMessage))
        {
            Msg->SetMsg(ImageMsg);
            ImagePublisher->Publish();
            // UE_LOG(LogTemp, Display, TEXT("Published image: %dx%d, %d bytes"), 
            //        ImageResolution.X, ImageResolution.Y, static_cast<int32>(ImageMsg.Data.Num()));
        }
    }
    else 
    {
        UE_LOG(LogTemp, Error, TEXT("ImagePublisher invalid: %d, Msg: %d"), 
            IsValid(ImagePublisher), 
            (ImagePublisher ? IsValid(ImagePublisher->TopicMessage) : false));
    }

    bIsProcessingImage = false;
}

void AROS2Controller::UpdatePositionMessage(UROS2GenericMsg* InMessage)
{
    if(!QuadPawn) return;

    FROSPoint PositionMsg;
    const FVector WorldPosition = QuadPawn->GetActorLocation();
    
    PositionMsg.X = WorldPosition.X;
    PositionMsg.Y = WorldPosition.Y;
    PositionMsg.Z = WorldPosition.Z;

    CastChecked<UROS2PointMsg>(InMessage)->SetMsg(PositionMsg);
}

void AROS2Controller::UpdateImageMessage(UROS2GenericMsg* InMessage)
{
    // Empty - now handled by timer-based capture system
}

void AROS2Controller::SetupObstacleManager()
{
    // Try to find existing obstacle manager
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AObstacleManager::StaticClass(), FoundActors);
    
    if (FoundActors.Num() > 0)
    {
        ObstacleManagerInstance = Cast<AObstacleManager>(FoundActors[0]);
        UE_LOG(LogTemp, Display, TEXT("ROS2Controller: Found existing ObstacleManager: %s"), 
            *ObstacleManagerInstance->GetName());
    }
    else
    {
        // Try to spawn a new obstacle manager
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        ObstacleManagerInstance = GetWorld()->SpawnActor<AObstacleManager>(AObstacleManager::StaticClass(), 
                                                                           FVector::ZeroVector, 
                                                                           FRotator::ZeroRotator, 
                                                                           SpawnParams);
        if (ObstacleManagerInstance)
        {
            UE_LOG(LogTemp, Display, TEXT("ROS2Controller: Spawned new ObstacleManager"));
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("ROS2Controller: Failed to spawn ObstacleManager"));
        }
    }

    if (ObstacleManagerInstance)
    {
        UE_LOG(LogTemp, Warning, TEXT("ObstacleManager status: Valid=%d, Name=%s"), 
            IsValid(ObstacleManagerInstance),
            *ObstacleManagerInstance->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("ObstacleManagerInstance is NULL after setup!"));
    }
}

void AROS2Controller::HandleObstacleMessage(const UROS2GenericMsg* InMsg)
{
    // Log every incoming message
    UE_LOG(LogTemp, Warning, TEXT("Obstacle message received!"));
    
    const UROS2Float64Msg* Float64Msg = Cast<UROS2Float64Msg>(InMsg);
    if (!Float64Msg)
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid message type!"));
        return;
    }
    
    FROSFloat64 RosMsg;
    Float64Msg->GetMsg(RosMsg);
    float ObstacleCount = RosMsg.Data;
    
    UE_LOG(LogTemp, Display, TEXT("Received obstacle count: %f"), ObstacleCount);
    
    // Only create obstacles if the count is positive and manager exists
    if (ObstacleCount > 0.0f && ObstacleManagerInstance)
    {
        int32 Count = FMath::RoundToInt(ObstacleCount); // Convert float to int
        UE_LOG(LogTemp, Warning, TEXT("Creating %d obstacles"), Count);
        ObstacleManagerInstance->CreateObstacles(Count, EGoalPosition::Random);
    }
}