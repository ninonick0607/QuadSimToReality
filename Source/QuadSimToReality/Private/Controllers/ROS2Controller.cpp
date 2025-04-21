#include "Controllers/ROS2Controller.h" 

#include "MaterialHLSLTree.h"
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
#include "Msgs/ROS2Odom.h"
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
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *OdometryTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, OdometryTopicName,          
        UROS2Publisher::StaticClass(),          
        UROS2OdomMsg::StaticClass(),        
        OdometryFrequencyHz,                    
        &AROS2Controller::UpdateOdometryMessage,
        UROS2QoS::Default,                      
        OdometryPublisher);
    
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *PositionGoalTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, PositionGoalTopicName, UROS2Publisher::StaticClass(), UROS2PointMsg::StaticClass(), 
        GoalFrequenzyHz, &AROS2Controller::UpdateGoalPositionMessage, UROS2QoS::Default, GoalPosition); // Try services if not working

    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *ImageTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, ImageTopicName, UROS2Publisher::StaticClass(), UROS2ImgMsg::StaticClass(), 
        ImageFrequencyHz, &AROS2Controller::UpdateImageMessage, UROS2QoS::SensorData, ImagePublisher);

    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *CollisionTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
        Node, this, CollisionTopicName,
        UROS2Publisher::StaticClass(), UROS2Float64Msg::StaticClass(),
        10, &AROS2Controller::UpdateCollisionMessage,
        UROS2QoS::Default, CollisionPublisher);

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
        Node, this, ResetTopicName,
        UROS2EmptyMsg::StaticClass(),
        &AROS2Controller::HandleResetCommand
    );

    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *HoverTopicName);
    ROS2_CREATE_SUBSCRIBER( 
        Node, this, HoverTopicName, UROS2Float64Msg::StaticClass(), &AROS2Controller::HandleHoverCommand);
    
    // ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
    //     Node, this, CollisionTopicName, UROS2Publisher::StaticClass(), UROS2Float64Msg::StaticClass(),
    //     PositionFrequencyHz, &AROS2Controller::UpdateCollisionMessage, UROS2QoS::Default, CollisionPublisher);

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

void AROS2Controller::HandleHoverCommand(const UROS2GenericMsg* InMsg)
{
    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleHoverMessage: InMsg is null")); return; }
    const UROS2Float64Msg* Float64MsgWrapper = Cast<UROS2Float64Msg>(InMsg); 
    FROSFloat64 HoverData; 
    Float64MsgWrapper->GetMsg(HoverData);
    const int32 HoverHeight = FMath::RoundToInt(HoverData.Data);
    UE_LOG(LogTemp, Log, TEXT("Received Hover Height: %d"), HoverHeight);

    UQuadDroneController* DroneController = QuadPawn->QuadController;
    DroneController->SetHoverMode(true, HoverHeight);
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
    LastReceivedObstacleCount = ObstacleCount; 

    UE_LOG(LogTemp, Log, TEXT("Received obstacle count: %d"), ObstacleCount);
    ObstacleManagerInstance->CreateObstacles(ObstacleCount, EGoalPosition::Random);
}

void AROS2Controller::HandleVelocityCommand(const UROS2GenericMsg* InMsg)
{
    // UE_LOG(LogTemp, Warning, TEXT("HandleVelocityCommand CALLBACK EXECUTED")); // Keep for debugging if needed

    if (!InMsg) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: InMsg is null")); return; }
    const UROS2TwistMsg* TwistMsgWrapper = Cast<UROS2TwistMsg>(InMsg);
    if (!TwistMsgWrapper) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: Invalid msg type")); return; }

    if (!IsValid(QuadPawn)) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: QuadPawn invalid")); return; }
    UQuadDroneController* DroneController = QuadPawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleVelocityCommand: QuadPawn->QuadController is invalid!")); return; }

    FROSTwist TwistData;
    TwistMsgWrapper->GetMsg(TwistData);

    const float M_TO_CM = 100.f;
    const float TargetLinearXVelocity_cms = TwistData.Linear.X * M_TO_CM;
    const float TargetLinearYVelocity_cms = TwistData.Linear.Y * M_TO_CM; // Use Y if needed in future
    const float TargetLinearZVelocity_cms = TwistData.Linear.Z * M_TO_CM; // Use Z if needed in future

    FVector DesiredVelocityVector = FVector(TargetLinearXVelocity_cms, TargetLinearYVelocity_cms, TargetLinearZVelocity_cms);
    DroneController->SetDesiredVelocity(DesiredVelocityVector);
    const float TargetAngularZ_radps = TwistData.Angular.Z;
    DroneController->SetDesiredYawRate(TargetAngularZ_radps);

}

void AROS2Controller::HandleResetCommand(const UROS2GenericMsg* InMsg)
{
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Processing 'reset' command (received Empty message)..."));

    if (!IsValid(QuadPawn)) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: QuadPawn invalid")); return; }
    UQuadDroneController* DroneController = QuadPawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleResetCommand: QuadPawn->QuadController invalid! Cannot reset.")); return; }

    UE_LOG(LogTemp, Log, TEXT("Calling DroneController->ResetDroneOrigin()"));
    DroneController->ResetDroneOrigin();
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Drone Reset via Controller Complete."));
}

void AROS2Controller::UpdateGoalPositionMessage(UROS2GenericMsg* InMessage)
{
    if (!InMessage) return;

    FROSPoint GoalData;
    const FVector GoalLocation = ObstacleManagerInstance->GetGoalPosition();
    
    GoalData.X = GoalLocation.X;
    GoalData.Y = GoalLocation.Y;
    GoalData.Z = GoalLocation.Z;
    
    if(UROS2PointMsg* GoalMsg = Cast<UROS2PointMsg>(InMessage))
    {
        GoalMsg->SetMsg(GoalData);
    }
    else { UE_LOG(LogTemp, Error, TEXT("UpdateGoalPositionMessage: Failed cast to UROS2PointMsg")); }
}

void AROS2Controller::UpdateCollisionMessage(UROS2GenericMsg* InMessage)
{
    if (!IsValid(QuadPawn) || !InMessage) return;
    UROS2Float64Msg* Msg = Cast<UROS2Float64Msg>(InMessage);
    if (!Msg) return;
    FROSFloat64 CollisionData;
    CollisionData.Data = QuadPawn->getCollisionState() ? 1.0 : 0.0;
    Msg->SetMsg(CollisionData);
}

void AROS2Controller::UpdateOdometryMessage(UROS2GenericMsg* InMessage)
{
    if (!IsValid(QuadPawn) || !InMessage || !IsValid(OdometryPublisher)) return;
    UQuadDroneController* DroneController = QuadPawn->QuadController;
    if (!IsValid(DroneController)) return;

    FROSOdom OdometryData;

    // Header
    FTimespan Time = FDateTime::UtcNow().GetTimeOfDay();
    OdometryData.Header.Stamp.Sec = static_cast<int32>(Time.GetTotalSeconds());
    OdometryData.Header.Stamp.Nanosec = static_cast<uint32>(Time.GetFractionNano());
    OdometryData.Header.FrameId = TEXT("odom");       // Pose is relative to the odom frame
    OdometryData.ChildFrameId = TEXT("base_link"); // Twist is relative to the base_link frame

    // Pose (in Odom Frame) - Convert CM to M
    const FVector WorldPositionCm = QuadPawn->GetActorLocation();
    const FQuat WorldOrientationQuat = DroneController->GetOrientationAsQuat(); // Already world orientation
    const float CM_TO_M = 0.01f;

    OdometryData.Pose.Pose.Position.X = WorldPositionCm.X * CM_TO_M;
    OdometryData.Pose.Pose.Position.Y = WorldPositionCm.Y * CM_TO_M;
    OdometryData.Pose.Pose.Position.Z = WorldPositionCm.Z * CM_TO_M;
    OdometryData.Pose.Pose.Orientation = WorldOrientationQuat; // Use the world orientation

    const FVector WorldLinearVelocityCmps = DroneController->GetCurrentVelocity(); // Get World Velocity
    const FRotator WorldRotation = QuadPawn->GetActorRotation();          // Get World Rotation

   const FVector LocalLinearVelocityCmps = WorldRotation.UnrotateVector(WorldLinearVelocityCmps);
    const FVector AngularVelocityRadps = DroneController->GetCurrentAngularVelocityRADPS(); // Assumed already local

    // Populate Twist with LOCAL Linear Velocity (Convert CM/s to M/s)
    OdometryData.Twist.Twist.Linear.X = LocalLinearVelocityCmps.X * CM_TO_M;
    OdometryData.Twist.Twist.Linear.Y = LocalLinearVelocityCmps.Y * CM_TO_M;
    OdometryData.Twist.Twist.Linear.Z = LocalLinearVelocityCmps.Z * CM_TO_M; // Make sure Z is included if needed

    // Populate Twist with Angular Velocity (already local, rad/s)
    OdometryData.Twist.Twist.Angular.X = AngularVelocityRadps.X;
    OdometryData.Twist.Twist.Angular.Y = AngularVelocityRadps.Y;
    OdometryData.Twist.Twist.Angular.Z = AngularVelocityRadps.Z;

    // Set message data
    if (UROS2OdomMsg* OdometryMsg = Cast<UROS2OdomMsg>(InMessage))
    {
        OdometryMsg->SetMsg(OdometryData);
    }
    else { UE_LOG(LogTemp, Error, TEXT("UpdateOdometryMessage: Failed cast to UROS2OdomMsg")); }
}

void AROS2Controller::InitializeImageCapture()
{
    if (!QuadPawn || !QuadPawn->CameraFPV || !SceneCapture)
    {
        UE_LOG(LogTemp, Error, TEXT("Missing required components for image capture!"));
        return;
    }

    if (!SceneCapture->IsAttachedTo(QuadPawn->CameraFPV))
    {
        SceneCapture->AttachToComponent(QuadPawn->CameraFPV, 
            FAttachmentTransformRules::SnapToTargetIncludingScale);
    }
    SceneCapture->RegisterComponent();
    SceneCapture->HiddenActors.Add(QuadPawn);

    for (int32 i = 0; i < 2; ++i)
    {
        RenderTargets[i] = NewObject<UTextureRenderTarget2D>(this);
        RenderTargets[i]->InitCustomFormat(
            ImageResolution.X,
            ImageResolution.Y,
            PF_B8G8R8A8, 
            true  
        );
        RenderTargets[i]->TargetGamma = 2.2f;
        RenderTargets[i]->UpdateResourceImmediate(true);
    }

    SceneCapture->FOVAngle = QuadPawn->CameraFPV->FieldOfView;
    SceneCapture->ShowFlags.SetTonemapper(true);
    SceneCapture->CaptureSource = SCS_FinalColorLDR;
    SceneCapture->bCaptureEveryFrame = false;
    SceneCapture->bAlwaysPersistRenderingState = true;
}

void AROS2Controller::CaptureImage()
{
    if (!SceneCapture || bIsProcessingImage) return;

    SceneCapture->SetWorldLocationAndRotation(
        QuadPawn->CameraFPV->GetComponentLocation(),
        QuadPawn->CameraFPV->GetComponentRotation()
    );

    const FVector CaptureLocation = QuadPawn->CameraFPV->GetComponentLocation();
    const FRotator CameraFullRotation = QuadPawn->CameraFPV->GetComponentRotation();

    const FRotator CaptureYawOnlyRotation = FRotator(0.0f, CameraFullRotation.Yaw, 0.0f);
    SceneCapture->SetWorldLocationAndRotation(CaptureLocation, CaptureYawOnlyRotation);
    
    CurrentRenderTargetIndex = (CurrentRenderTargetIndex + 1) % 2;
    UTextureRenderTarget2D* CurrentTarget = RenderTargets[CurrentRenderTargetIndex];
    
    SceneCapture->TextureTarget = CurrentTarget;
    SceneCapture->CaptureScene();

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

    FROSImg ImageMsg;
    ImageMsg.Height = ImageResolution.Y;
    ImageMsg.Width = ImageResolution.X;
    ImageMsg.Encoding = "bgr8";
    ImageMsg.Step = ImageResolution.X * 3;
    ImageMsg.Data.Reserve(Pixels.Num() * 3);

    for (const FColor& Pixel : Pixels)
    {
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

void AROS2Controller::UpdateImageMessage(UROS2GenericMsg* InMessage)
{
}
// Accessor implementations for UI
FVector AROS2Controller::GetCurrentGoalPosition() const
{
    return ObstacleManagerInstance ? ObstacleManagerInstance->GetGoalPosition() : FVector::ZeroVector;
}

FString AROS2Controller::GetDroneID() const
{
    return QuadPawn ? QuadPawn->DroneID : FString(TEXT("Unknown"));
}