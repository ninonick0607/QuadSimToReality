#include "Controllers/ROS2Controller.h"
#include "ROS2Node.h"
#include "Kismet/GameplayStatics.h"
#include "Async/Async.h"

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

    // Initialize ROS2 node
    Node->Init();

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
    ImageMsg.Encoding = "rgb8";
    ImageMsg.Step = ImageResolution.X * 3;
    ImageMsg.Data.Reserve(Pixels.Num() * 3);

    for (const FColor& Pixel : Pixels)
    {
        ImageMsg.Data.Add(Pixel.R);  
        ImageMsg.Data.Add(Pixel.G);  
        ImageMsg.Data.Add(Pixel.B);  
    }

    if (IsValid(ImagePublisher) && IsValid(ImagePublisher->TopicMessage))
    {
        if (UROS2ImgMsg* Msg = Cast<UROS2ImgMsg>(ImagePublisher->TopicMessage))
        {
            Msg->SetMsg(ImageMsg);
            ImagePublisher->Publish();
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
    
    if(++UpdateCount % 10 == 0)  
    {
        UE_LOG(LogTemp, Display, TEXT("Position Update: X=%.2f, Y=%.2f, Z=%.2f"), 
            WorldPosition.X, WorldPosition.Y, WorldPosition.Z);
    }
}

void AROS2Controller::UpdateImageMessage(UROS2GenericMsg* InMessage)
{
    // Empty - now handled by timer-based capture system
}