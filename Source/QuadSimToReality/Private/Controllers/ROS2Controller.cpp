#include "Controllers/ROS2Controller.h" 
#include "Kismet/GameplayStatics.h"
#include "Async/Async.h" 
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TimerManager.h"
#include "RHICommandList.h"
#include "RenderingThread.h" 
#include "Camera/CameraComponent.h"

#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "ROS2Subscriber.h"
#include "Controllers/QuadDroneController.h"

#include "Msgs/ROS2Point.h"     
#include "Msgs/ROS2Img.h"       
#include "Msgs/ROS2Float64.h"   
#include "Msgs/ROS2Twist.h"     
#include "Msgs/ROS2Str.h"       
#include "Msgs/ROS2GenericMsg.h"

#include "Pawns/QuadPawn.h"     
#include "Utility/ObstacleManager.h"
#include "Controllers/QuadDroneController.h"

AROS2Controller::AROS2Controller()
{
    PrimaryActorTick.bCanEverTick = false;
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    SceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
    RenderTargets.SetNum(2);
}

void AROS2Controller::BeginPlay()
{
    Super::BeginPlay();

    if (!IsValid(QuadPawn))
    {
        UE_LOG(LogTemp, Error, TEXT("AROS2Controller::BeginPlay - QuadPawn reference not set! Aborting."));
        return;
    }
    if (!IsValid(QuadPawn->CameraFPV)) 
    {
        UE_LOG(LogTemp, Error, TEXT("AROS2Controller::BeginPlay - QuadPawn->CameraFPV is not valid! Aborting."));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Initializing ROS2 Node '%s' in namespace '%s'"), *NodeName, *Namespace);
    Node->Name = NodeName;
    Node->Namespace = Namespace;
    Node->Init();

    // --- Setup Publishers ---
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *PositionTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, PositionTopicName, UROS2Publisher::StaticClass(), UROS2PointMsg::StaticClass(), 
        PositionFrequencyHz, &AROS2Controller::UpdatePositionMessage, UROS2QoS::Default, PositionPublisher); // Try Services if not working
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *PositionGoalTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, PositionGoalTopicName, UROS2Publisher::StaticClass(), UROS2PointMsg::StaticClass(), 
        GoalFrequenzyHz, &AROS2Controller::UpdateGoalPositionMessage, UROS2QoS::Default, GoalPosition); // Try services if not working
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *ImageTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, ImageTopicName, UROS2Publisher::StaticClass(), UROS2ImgMsg::StaticClass(), 
        ImageFrequencyHz, &AROS2Controller::UpdateImageMessage, UROS2QoS::SensorData, ImagePublisher);

    // --- Setup Obstacle Manager ---
    SetupObstacleManager();
    
    // --- Setup Subscribers ---
    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *ObstacleTopicName);
    ROS2_CREATE_SUBSCRIBER( 
        Node, this, ObstacleTopicName, UROS2Float64Msg::StaticClass(), &AROS2Controller::HandleObstacleMessage);

    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *CmdVelTopicName);
    ROS2_CREATE_SUBSCRIBER( 
        Node, this, CmdVelTopicName, UROS2TwistMsg::StaticClass(), &AROS2Controller::HandleVelocityCommand);

    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *ResetTopicName);
    ROS2_CREATE_SUBSCRIBER( 
        Node, this, ResetTopicName, UROS2StrMsg::StaticClass(), &AROS2Controller::HandleResetCommand);

    InitializeImageCapture();
    if (ImageFrequencyHz > 0 && GetWorld()) {
        UE_LOG(LogTemp, Log, TEXT("Starting image capture timer (Interval: %.4f s)"), 1.0f / ImageFrequencyHz);
        GetWorld()->GetTimerManager().SetTimer(CaptureTimerHandle, this, &AROS2Controller::CaptureImage, 1.0f / ImageFrequencyHz, true);
    } else if (ImageFrequencyHz <= 0) { UE_LOG(LogTemp, Warning, TEXT("Image capture timer not started (Frequency <= 0).")); }

    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller initialization complete."));
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
    //SceneCapture->PostProcessSettings = QuadPawn->CameraFPV->PostProcessSettings;
    SceneCapture->ShowFlags.SetTonemapper(true);
    // SceneCapture->ShowFlags.SetColorGrading(false);
    // SceneCapture->ShowFlags.SetEyeAdaptation(false);
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
    if (!IsValid(QuadPawn)) return;
    if (!InMessage) return; 

    FROSPoint PositionData; 
    const FVector WorldPosition = QuadPawn->GetActorLocation();
    const float ScaleFactor = 1.f;//0.01f; // cm to m

    PositionData.X = WorldPosition.X * ScaleFactor;
    PositionData.Y = WorldPosition.Y * ScaleFactor;
    PositionData.Z = WorldPosition.Z * ScaleFactor;

    if (UROS2PointMsg* PointMsg = Cast<UROS2PointMsg>(InMessage))
    {
        PointMsg->SetMsg(PositionData);
    }
    else { UE_LOG(LogTemp, Error, TEXT("UpdatePositionMessage: Failed cast to UROS2PointMsg")); }
}

// void AROS2Controller::UpdateGoalPositionMessage(UROS2GenericMsg* InMessage)
// {
//
//     FROSPoint GoalMsg;
//     const FVector GoalLocation = ObstacleManagerInstance->GetActorLocation();
//
//     GoalMsg.X = GoalLocation.X;
//     GoalMsg.Y = GoalLocation.Y;
//     GoalMsg.Z = GoalLocation.Z;
//
//     CastChecked<UROS2PointMsg>(InMessage)->SetMsg(GoalMsg);
// }

void AROS2Controller::UpdateGoalPositionMessage(UROS2GenericMsg* InMessage)
{
    if (!InMessage) return;

    FROSPoint GoalData; 
    const float ScaleFactor = 1.f;//0.01f;

    GoalData.X = 0.0;
    GoalData.Y = 0.0;
    GoalData.Z = TargetAltitude * ScaleFactor; // Use configured altitude, convert to meters

    if(UROS2PointMsg* GoalMsg = Cast<UROS2PointMsg>(InMessage))
    {
        GoalMsg->SetMsg(GoalData);
    }
    else { UE_LOG(LogTemp, Error, TEXT("UpdateGoalPositionMessage: Failed cast to UROS2PointMsg")); }
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
        UE_LOG(LogTemp, Error, TEXT("ObstacleManagerInstance .0is NULL after setup!"));
    }
}

void AROS2Controller::HandleObstacleMessage(const UROS2GenericMsg* InMsg)
{
    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleObstacleMessage: InMsg is null")); return; }
    const UROS2Float64Msg* Float64MsgWrapper = Cast<UROS2Float64Msg>(InMsg); 
    if (!Float64MsgWrapper) { UE_LOG(LogTemp, Error, TEXT("HandleObstacleMessage: Invalid msg type")); return; }

    if (!IsValid(ObstacleManagerInstance)) { UE_LOG(LogTemp, Error, TEXT("HandleObstacleMessage: ObstacleManager invalid")); return; }

    FROSFloat64 ObstacleData; 
    Float64MsgWrapper->GetMsg(ObstacleData);
    const int32 ObstacleCount = FMath::RoundToInt(ObstacleData.Data);

    UE_LOG(LogTemp, Log, TEXT("Received obstacle count: %d"), ObstacleCount);
    ObstacleManagerInstance->CreateObstacles(ObstacleCount, EGoalPosition::Random);
}

void AROS2Controller::HandleVelocityCommand(const UROS2GenericMsg* InMsg)
{
    UE_LOG(LogTemp, Warning, TEXT("!!! HandleVelocityCommand CALLBACK EXECUTED !!!")); // <-- ADD THIS LINE

    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: InMsg is null")); return; }
    const UROS2TwistMsg* TwistMsgWrapper = Cast<UROS2TwistMsg>(InMsg);
    if (!TwistMsgWrapper) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: Invalid msg type")); return; }

    if (!IsValid(QuadPawn)) { return; }
    UQuadDroneController* DroneController = QuadPawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleVelocityCommand: QuadPawn->QuadController is invalid!")); return; }

    FROSTwist TwistData;
    TwistMsgWrapper->GetMsg(TwistData);
    const float MPS_TO_CMPS = 1.f; // Assuming incoming is cm/s
    const float TargetLinearZVelocity_cms = TwistData.Linear.Z * MPS_TO_CMPS;
    const float TargetLinearXVelocity_cms = 0.0f;
    const float TargetLinearYVelocity_cms = 0.0f;
    FVector DesiredVelocityVector = FVector(TargetLinearXVelocity_cms, TargetLinearYVelocity_cms, TargetLinearZVelocity_cms);
    // UE_LOG(LogTemp, Log, TEXT("Setting Desired Velocity via Controller: %s (cm/s)"), *DesiredVelocityVector.ToString());
    DroneController->SetDesiredVelocity(DesiredVelocityVector);
}

void AROS2Controller::HandleResetCommand(const UROS2GenericMsg* InMsg)
{
    UE_LOG(LogTemp, Warning, TEXT("!!! HandleResetCommand CALLBACK EXECUTED !!!")); 

    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: InMsg is null")); return; }
    const UROS2StrMsg* StringMsgWrapper = Cast<UROS2StrMsg>(InMsg);
    if (!StringMsgWrapper) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: Invalid msg type")); return; }

    FROSStr StringData;
    StringMsgWrapper->GetMsg(StringData);

    if (StringData.Data.Equals(TEXT("reset"), ESearchCase::IgnoreCase))
    {
        UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Processing 'reset' command...")); // Changed log level

        if (!IsValid(QuadPawn)) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: QuadPawn invalid")); return; }
        UQuadDroneController* DroneController = QuadPawn->QuadController;
        if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleResetCommand: QuadPawn->QuadController invalid! Cannot reset.")); return; }

        UE_LOG(LogTemp, Log, TEXT("Calling DroneController->ResetDroneOrigin()"));
        DroneController->ResetDroneOrigin();
        UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Drone Reset via Controller Complete."));
    }
    // else { UE_LOG(LogTemp, Verbose, TEXT("Received Str on ResetTopic != 'reset': %s"), *StringData.Data); }
}

void AROS2Controller::UpdateImageMessage(UROS2GenericMsg* InMessage)
{
    // Empty - now handled by timer-based capture system
}