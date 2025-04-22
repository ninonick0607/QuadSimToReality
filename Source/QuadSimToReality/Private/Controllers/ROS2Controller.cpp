
#include "Controllers/ROS2Controller.h"

// --- Standard Includes ---
#include "Kismet/GameplayStatics.h"
#include "Async/Async.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TimerManager.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Camera/CameraComponent.h"

// --- ROS 2 Includes ---
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
#include "ROS2Subscriber.h"
#include "rclcUtilities.h" // Brings in UROS2Utils

// Message Headers for Wrappers (UROS2...Msg) and FStructs (FROS...)
#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2TFMsg.h"         // Defines UROS2TFMsgMsg and FROSTFMsg
#include "Msgs/ROS2TFStamped.h"    // Defines UROS2TFStampedMsg and FROSTFStamped
#include "Msgs/ROS2PoseStamped.h"  // Defines UROS2PoseStampedMsg and FROSPoseStamped
#include "Msgs/ROS2Odom.h"
#include "Msgs/ROS2Point.h"        // Defines UROS2PointMsg and FROSPoint
#include "Msgs/ROS2Img.h"
#include "Msgs/ROS2Float64.h"
#include "Msgs/ROS2Twist.h"
#include "Msgs/ROS2Str.h"
#include "Msgs/ROS2Empty.h"
#include "Msgs/ROS2TF.h"          // Defines FROSTransform (used implicitly by FROSTFStamped)
#include "Msgs/ROS2Header.h"      // Defines FROSStdHeader
#include "Msgs/ROS2Time.h"        // Defines FROSTime
#include "Msgs/ROS2Pose.h"        // Defines FROSPose
#include "Msgs/ROS2Quat.h"        // Defines UROS2QuatMsg and FROSQuat

// C-Struct Headers (Needed for utility function inputs & low-level assignment)
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"
#include "rosidl_runtime_c/string_functions.h" // For string assign/init/fini

// --- Project Specific Includes ---
#include "Pawns/QuadPawn.h"
#include "Utility/ObstacleManager.h"
#include "Controllers/QuadDroneController.h" // Assuming QuadPawn uses this
// *** Include your actual Navigation Component header ***
#include "Utility/NavigationComponent.h" // Make sure this path is correct



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

    // --- Pawn Validation ---
    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!IsValid(Pawn))
    {
        UE_LOG(LogTemp, Error, TEXT("AROS2Controller::BeginPlay - Owning QuadPawn not found! Aborting."));
        return;
    }
    bool bCanCaptureImages = IsValid(Pawn->CameraFPV);
    if (!bCanCaptureImages)
    {
        UE_LOG(LogTemp, Warning, TEXT("AROS2Controller::BeginPlay - Pawn->CameraFPV is not valid. Image capture will be disabled."));
    }

    // --- Node Initialization ---
    FString UniqueNodeName = NodeName;
    if (Pawn) UniqueNodeName = NodeName + TEXT("_") + Pawn->GetFName().ToString();
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller: Initializing ROS2 Node '%s' in namespace '%s'"), *UniqueNodeName, *Namespace);
    Node->Name = UniqueNodeName;
    Node->Namespace = Namespace;
    Node->Init();

    // --- Setup Publishers ---
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *OdometryTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS( Node, this, OdometryTopicName, UROS2Publisher::StaticClass(), UROS2OdomMsg::StaticClass(), OdometryFrequencyHz, &AROS2Controller::UpdateOdometryMessage, UROS2QoS::Default, OdometryPublisher);
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *PositionGoalTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS( Node, this, PositionGoalTopicName, UROS2Publisher::StaticClass(), UROS2PointMsg::StaticClass(), GoalFrequenzyHz, &AROS2Controller::UpdateGoalPositionMessage, UROS2QoS::Default, GoalPosition);
    if (bCanCaptureImages) {
        UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *ImageTopicName);
        ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS( Node, this, ImageTopicName, UROS2Publisher::StaticClass(), UROS2ImgMsg::StaticClass(), ImageFrequencyHz, &AROS2Controller::UpdateImageMessage, UROS2QoS::SensorData, ImagePublisher);
    }
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *CollisionTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS( Node, this, CollisionTopicName, UROS2Publisher::StaticClass(), UROS2Float64Msg::StaticClass(), 10, &AROS2Controller::UpdateCollisionMessage, UROS2QoS::Default, CollisionPublisher);

    // TF Publisher (**USING CORRECT CLASS NAME**)
    UE_LOG(LogTemp, Log, TEXT("Setting up Publisher: %s"), *TFTopicName);
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(
      Node, this, TFTopicName, UROS2Publisher::StaticClass(),
      UROS2TFMsgMsg::StaticClass(), // *** Use the actual class name from ROS2TFMsg.h ***
      TFFrequencyHz, &AROS2Controller::UpdateTFMessage, UROS2QoS::Default, TfPublisher);

    // --- Setup Obstacle Manager ---
    SetupObstacleManager();

    // --- Setup Subscribers ---
    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *ObstacleTopicName);
    ROS2_CREATE_SUBSCRIBER(
      Node,
      this,
      ObstacleTopicName,
      UROS2Float64Msg::StaticClass(),
      &AROS2Controller::HandleObstacleMessage
    );
    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *CmdVelTopicName);
    ROS2_CREATE_SUBSCRIBER( Node, this, CmdVelTopicName, UROS2TwistMsg::StaticClass(), &AROS2Controller::HandleVelocityCommand);
    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *ResetTopicName);
    ROS2_CREATE_SUBSCRIBER( Node, this, ResetTopicName, UROS2EmptyMsg::StaticClass(), &AROS2Controller::HandleResetCommand);
    UE_LOG(LogTemp, Log, TEXT("Setting up Subscriber: %s"), *HoverTopicName);
    ROS2_CREATE_SUBSCRIBER( Node, this, HoverTopicName, UROS2Float64Msg::StaticClass(), &AROS2Controller::HandleHoverCommand);

    // --- Image Capture Initialization ---
    if (bCanCaptureImages) {
        InitializeImageCapture();
        if (ImageFrequencyHz > 0 && GetWorld()) {
            GetWorld()->GetTimerManager().SetTimer(CaptureTimerHandle, this, &AROS2Controller::CaptureImage, 1.0f / ImageFrequencyHz, true);
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller initialization complete for %s."), *Pawn->GetName());
}

void AROS2Controller::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (GetWorld()) // Check if World is valid
    {
        GetWorld()->GetTimerManager().ClearTimer(CaptureTimerHandle);
    }

    Super::EndPlay(EndPlayReason);
    UE_LOG(LogTemp, Warning, TEXT("AROS2Controller EndPlay called."));
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

    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn) { UE_LOG(LogTemp, Error, TEXT("HandleHoverCommand: Owning QuadPawn invalid")); return; }
    UQuadDroneController* DroneController = Pawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleHoverCommand: Pawn->QuadController invalid")); return; }
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

    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn) { UE_LOG(LogTemp, Error, TEXT("HandleVelocityCommand: Owning QuadPawn invalid")); return; }
    UQuadDroneController* DroneController = Pawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleVelocityCommand: Pawn->QuadController invalid")); return; }

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

    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn) { UE_LOG(LogTemp, Error, TEXT("HandleResetCommand: Owning QuadPawn invalid")); return; }
    UQuadDroneController* DroneController = Pawn->QuadController;
    if (!IsValid(DroneController)) { UE_LOG(LogTemp, Warning, TEXT("HandleResetCommand: Pawn->QuadController invalid! Cannot reset.")); return; }

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
    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn || !InMessage) return;
    UROS2Float64Msg* Msg = Cast<UROS2Float64Msg>(InMessage);
    if (!Msg) return;
    FROSFloat64 CollisionData;
    CollisionData.Data = Pawn->getCollisionState() ? 1.0 : 0.0;
    Msg->SetMsg(CollisionData);
}

void AROS2Controller::UpdateOdometryMessage(UROS2GenericMsg* InMessage)
{
    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn || !InMessage || !IsValid(OdometryPublisher)) return;
    UQuadDroneController* DroneController = Pawn->QuadController;
    if (!IsValid(DroneController)) return;

    FROSOdom OdometryData;

    // Header
    FTimespan Time = FDateTime::UtcNow().GetTimeOfDay();
    OdometryData.Header.Stamp.Sec = static_cast<int32>(Time.GetTotalSeconds());
    OdometryData.Header.Stamp.Nanosec = static_cast<uint32>(Time.GetFractionNano());
    OdometryData.Header.FrameId = TEXT("odom");       // Pose is relative to the odom frame
    OdometryData.ChildFrameId = TEXT("base_link"); // Twist is relative to the base_link frame

    // Pose (in Odom Frame) - Convert CM to M
    const FVector WorldPositionCm = Pawn->GetActorLocation();
    const FQuat WorldOrientationQuat = DroneController->GetOrientationAsQuat(); // Already world orientation
    const float CM_TO_M = 0.01f;

    OdometryData.Pose.Pose.Position.X = WorldPositionCm.X * CM_TO_M;
    OdometryData.Pose.Pose.Position.Y = WorldPositionCm.Y * CM_TO_M;
    OdometryData.Pose.Pose.Position.Z = WorldPositionCm.Z * CM_TO_M;
    OdometryData.Pose.Pose.Orientation = WorldOrientationQuat; // Use the world orientation

    const FVector WorldLinearVelocityCmps = DroneController->GetCurrentVelocity(); // Get World Velocity
    const FRotator WorldRotation = Pawn->GetActorRotation();          // Get World Rotation

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
    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn || !Pawn->CameraFPV || !SceneCapture)
    {
        UE_LOG(LogTemp, Error, TEXT("Missing required components for image capture!"));
        return;
    }

    if (!SceneCapture->IsAttachedTo(Pawn->CameraFPV))
    {
        SceneCapture->AttachToComponent(Pawn->CameraFPV,
            FAttachmentTransformRules::SnapToTargetIncludingScale);
    }
    SceneCapture->RegisterComponent();
    SceneCapture->HiddenActors.Add(Pawn);

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

    SceneCapture->FOVAngle = Pawn->CameraFPV->FieldOfView;
    SceneCapture->ShowFlags.SetTonemapper(true);
    SceneCapture->CaptureSource = SCS_FinalColorLDR;
    SceneCapture->bCaptureEveryFrame = false;
    SceneCapture->bAlwaysPersistRenderingState = true;
}

void AROS2Controller::CaptureImage()
{
    if (!SceneCapture || bIsProcessingImage) return;
    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!Pawn || !Pawn->CameraFPV) return;

    SceneCapture->SetWorldLocationAndRotation(
        Pawn->CameraFPV->GetComponentLocation(),
        Pawn->CameraFPV->GetComponentRotation()
    );

    const FVector CaptureLocation = Pawn->CameraFPV->GetComponentLocation();
    const FRotator CameraFullRotation = Pawn->CameraFPV->GetComponentRotation();

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
    {
        AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
        return Pawn ? Pawn->DroneID : FString(TEXT("Unknown"));
    }
}

void AROS2Controller::UpdateTFMessage(UROS2GenericMsg* InMsg)
{
    // 1) Cast to the actual wrapper class name from ROS2TFMsg.h
    auto* TfMsg = Cast<UROS2TFMsgMsg>(InMsg); // *** Use UROS2TFMsgMsg ***
    if (!TfMsg) return;

    AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor());
    if (!IsValid(Pawn)) return;

    // 2) Build the ROS C-struct TransformStamped
    geometry_msgs__msg__TransformStamped tf_stamped;
    if (!geometry_msgs__msg__TransformStamped__init(&tf_stamped)) {
        UE_LOG(LogTemp, Error, TEXT("UpdateTFMessage: Failed to init TransformStamped C-struct"));
        return;
    }

    // Auto-cleanup for the C-struct
    struct FScopeFiniGuard {
        geometry_msgs__msg__TransformStamped* Ptr;
        FScopeFiniGuard(geometry_msgs__msg__TransformStamped* InPtr) : Ptr(InPtr) {}
        ~FScopeFiniGuard() { if(Ptr) geometry_msgs__msg__TransformStamped__fini(Ptr); }
    } TfGuard(&tf_stamped);

    // Populate Header
    FTimespan Time = FDateTime::UtcNow().GetTimeOfDay();
    tf_stamped.header.stamp.sec = static_cast<int32>(Time.GetTotalSeconds());
    tf_stamped.header.stamp.nanosec = static_cast<uint32>(Time.GetFractionNano());
    if (!rosidl_runtime_c__String__assign(&tf_stamped.header.frame_id, TCHAR_TO_UTF8(*FString("odom")))) {
       UE_LOG(LogTemp, Error, TEXT("UpdateTFMessage: Failed to assign header.frame_id")); return;
    }
    if (!rosidl_runtime_c__String__assign(&tf_stamped.child_frame_id, TCHAR_TO_UTF8(*FString("base_link")))) {
       UE_LOG(LogTemp, Error, TEXT("UpdateTFMessage: Failed to assign child_frame_id")); return;
    }

    // Populate Transform using Utility Function
    tf_stamped.transform = UROS2Utils::TransformUEToROS(Pawn->GetActorTransform());

    // 3) Convert C-struct into the UE FStruct wrapper using the CONFIRMED method
    FROSTFStamped ue_stamp;
    ue_stamp.SetFromROS2(tf_stamped); // This method name is confirmed from ROS2TFStamped.h

    // (Fini will be called by TfGuard)

    // 4) Prepare the main message FStruct
    FROSTFMsg ue_msg;
    ue_msg.Transforms.Add(ue_stamp);

    // 5) Call SetMsg on the correct wrapper object type
    // If the "Cannot convert FROSTFMsg to FROSTF" error still occurs HERE,
    // it indicates a deeper issue possibly within RCLUE's handling of UROS2TFMsgMsg.
    TfMsg->SetMsg(ue_msg);
}
void AROS2Controller::HandleGoalPose(const UROS2GenericMsg* InMsg)
{
    auto* PoseWrap = Cast<UROS2PoseStampedMsg>(InMsg);
    if (!PoseWrap) return;

    // unwrap the ROS message
    FROSPoseStamped rosPose;
    PoseWrap->GetMsg(rosPose);

    // directly grab the UE FVector (no manual .X/.Y/.Z needed)
    FVector goal_loc = rosPose.Pose.Position;

    UE_LOG(LogTemp, Log, TEXT("New ROS goal at %s"), *goal_loc.ToString());

    // feed it to your NavComponent
    if (AQuadPawn* Pawn = Cast<AQuadPawn>(GetAttachParentActor()))
    {
        if (auto* Nav = Pawn->NavigationComponent)
        {
            Nav->SetCurrentDestination(goal_loc);
        }
    }
}
