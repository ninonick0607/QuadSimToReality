#include "Controllers/ZMQController.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "ImageUtils.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "Camera/CameraComponent.h"
#include "HAL/RunnableThread.h"
#include "Async/Async.h"
#include "Utility/ObstacleManager.h"

#include "Kismet/GameplayStatics.h"

AZMQController::AZMQController()
    : bIsCapturing(false)
    , bIsProcessingCommand(false)
    , DronePawn(nullptr)
    , DroneController(nullptr)
    , CaptureComponent(nullptr)
    , RenderTarget(nullptr)
    , CurrentGoalPosition(FVector(0.0f, 0.0f, 1000.0f))
    , TargetPawn(nullptr)
    , ObstacleManagerInstance(nullptr)  

{
    PrimaryActorTick.bCanEverTick = true;
}

AZMQController::~AZMQController()
{
}

void AZMQController::Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FZMQConfiguration& Config)
{
    UE_LOG(LogTemp, Display, TEXT("ZMQController::Initialize called with Pawn=%s, Controller=%s"),
           InPawn ? *InPawn->GetName() : TEXT("nullptr"),
           InDroneController ? *InDroneController->GetName() : TEXT("nullptr"));    

    Configuration = Config;
    // Overwrite the DroneID with the unique name from the pawn
    if (InPawn)
    {
        Configuration.DroneID = InPawn->GetName();
    }

    DronePawn = InPawn;
    DroneController = InDroneController;
    TargetPawn = InPawn;

    UE_LOG(LogTemp, Display, TEXT("ZMQController initialized with DroneID: %s"), *Configuration.DroneID);

    InitializeZMQ();
    InitializeImageCapture();
    
}

void AZMQController::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Display, TEXT("AZMQController BeginPlay: Creating separate window..."));
    // Auto-assign TargetPawn from attached parent pawn if not explicitly set
    if (!TargetPawn)
    {
        if (AQuadPawn* ParentPawn = Cast<AQuadPawn>(GetAttachParentActor()))
        {
            TargetPawn = ParentPawn;
            UE_LOG(LogTemp, Display, TEXT("AZMQController: Auto-assigned TargetPawn to %s"), *TargetPawn->GetName());
        }
    }
    if (TargetPawn)
    {
        if (TargetPawn->QuadController)
        {
            DronePawn = TargetPawn;
            DroneController = TargetPawn->QuadController;
            // Update DroneID to match pawn name
            Configuration.DroneID = TargetPawn->GetName();
            Initialize(DronePawn, DroneController, Configuration);
        }
        else
        {
            // Delay initialization until pawn's QuadController is ready
            GetWorld()->GetTimerManager().SetTimerForNextTick([this]()
            {
                this->CheckAndInitialize();
            });
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("AZMQController: No TargetPawn found; ZMQ initialization deferred"));
    }

    UE_LOG(LogTemp, Display, TEXT("Initial goal position set to: X=%f, Y=%f, Z=%f"),
           CurrentGoalPosition.X, CurrentGoalPosition.Y, CurrentGoalPosition.Z);

    if (Configuration.CaptureInterval > 0.0f)
    {
        GetWorld()->GetTimerManager().SetTimer(
            ImageCaptureTimerHandle,
            this,
            &AZMQController::ProcessImageCapture,
            Configuration.CaptureInterval,
            true
        );
    }

    // Check and initialize the ObstacleManager
    if (!ObstacleManagerInstance) {
        TArray<AActor*> FoundActors;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AObstacleManager::StaticClass(), FoundActors);
        
        if (FoundActors.Num() > 0) {
            ObstacleManagerInstance = Cast<AObstacleManager>(FoundActors[0]);
            UE_LOG(LogTemp, Display, TEXT("Found ObstacleManager: %s"), 
                *ObstacleManagerInstance->GetName());
        } else {
            UE_LOG(LogTemp, Warning, TEXT("No ObstacleManager found in level! Will try to spawn one."));
            
            // Try to spawn an ObstacleManager if none exists
            FActorSpawnParameters SpawnParams;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            ObstacleManagerInstance = GetWorld()->SpawnActor<AObstacleManager>(AObstacleManager::StaticClass(), 
                                                                               FVector::ZeroVector, 
                                                                               FRotator::ZeroRotator, 
                                                                               SpawnParams);
            if (ObstacleManagerInstance) {
                UE_LOG(LogTemp, Display, TEXT("Spawned new ObstacleManager"));
            } else {
                UE_LOG(LogTemp, Error, TEXT("Failed to spawn ObstacleManager"));
                return;
            }
        }
    }

}



void AZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorld()->GetTimerManager().ClearTimer(ImageCaptureTimerHandle);
    
    PublishSocket.Reset();
    CommandSocket.Reset();
    ControlSocket.Reset();
    CollisionSocket.Reset();
    
    Super::EndPlay(EndPlayReason);
}

void AZMQController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ProcessCommands();
    SendStateData();
}

void AZMQController::InitializeZMQ()
{
    try 
    {
        PublishSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::pub);
        FString PublishEndpoint = FString::Printf(TEXT("tcp://*:%d"), Configuration.PublishPort);
        PublishSocket->bind(TCHAR_TO_UTF8(*PublishEndpoint));

        CommandSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::sub);
        FString CommandEndpoint = FString::Printf(TEXT("tcp://localhost:%d"), Configuration.CommandPort);
        CommandSocket->connect(TCHAR_TO_UTF8(*CommandEndpoint));
        CommandSocket->set(zmq::sockopt::subscribe, "");

        ControlSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::pub);
        FString ControlEndpoint = FString::Printf(TEXT("tcp://*:%d"), Configuration.ControlPort);
        ControlSocket->bind(TCHAR_TO_UTF8(*ControlEndpoint));

        ObstacleSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::sub);
        FString ObstacleEndpoint = FString::Printf(TEXT("tcp://localhost:%d"), Configuration.ObstaclePort);
        ObstacleSocket->connect(TCHAR_TO_UTF8(*ObstacleEndpoint));
        ObstacleSocket->set(zmq::sockopt::subscribe, "");

        CollisionSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::pub);
        FString CollisionEndpoint = FString::Printf(TEXT("tcp://*:%d"), Configuration.CollisionPort);
        CollisionSocket->bind(TCHAR_TO_UTF8(*CollisionEndpoint));
        
        UE_LOG(LogTemp, Display, TEXT("ZMQ Initialization Successful"));
    }
    catch (const zmq::error_t& Error)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Initialization Error: %s"), 
               *FString(UTF8_TO_TCHAR(Error.what())));
    }
}

void AZMQController::ProcessCommands()
{
    if (bIsProcessingCommand) return;
    bIsProcessingCommand = true;

    try
    {
        // Check Command Socket
        if (CommandSocket)
        {
            zmq::multipart_t CommandMessage;
            if (CommandMessage.recv(*CommandSocket, static_cast<int>(zmq::recv_flags::dontwait)))
            {
                if (!CommandMessage.empty())
                {
                    std::string Command = CommandMessage.popstr();

                    if (Command == "RESET")
                    {
                        HandleResetCommand();
                    }
                    else if (Command == "INTEGRAL_RESET" && DroneController)
                    {
                        DroneController->ResetDroneIntegral();
                    }
                    else if (Command == "VELOCITY")
                    {
                        UE_LOG(LogTemp, Display, TEXT("[ZMQController] Received 'VELOCITY' command."));
                        HandleVelocityCommand(CommandMessage);
                    }
                }
            }
        }

        if (ObstacleSocket)
        {
            zmq::multipart_t ObstacleMessage;
            if (ObstacleMessage.recv(*ObstacleSocket, static_cast<int>(zmq::recv_flags::dontwait)))
            {
                if (!ObstacleMessage.empty())
                {
                    std::string Command = ObstacleMessage.popstr();
                    if (Command == "CREATE_OBSTACLE")
                    {
                        UE_LOG(LogTemp, Display, TEXT("Creating Obstacles.... Received on port %d"), Configuration.ObstaclePort);
                        HandleObstacleCommand(ObstacleMessage);
                    }
                }
            }
        }
    }
    catch (const zmq::error_t& Error)
    {
        UE_LOG(LogTemp, Warning, TEXT("Command processing error: %s"),
               *FString(UTF8_TO_TCHAR(Error.what())));
    }

    bIsProcessingCommand = false;
}

void AZMQController::HandleResetCommand()
{
    if (!DroneController) return;

    // Set new goal position (varying only Z height)
    CurrentGoalPosition = FVector(0.0f, 0.0f, 500);

    UE_LOG(LogTemp, Warning, TEXT("ZMQController: New goal height set to: Z=%f"), CurrentGoalPosition.Z);
    UE_LOG(LogTemp, Warning, TEXT("Reset command sent over "));
    DroneController->ResetDroneOrigin();
}

void AZMQController::HandleVelocityCommand(zmq::multipart_t& Message)
{
    if (!DroneController || Message.empty()) return;

    zmq::message_t VelocityData = Message.pop();
    if (VelocityData.size() == sizeof(float) * 4 && VelocityData.data())
    {
        float* VelocityArray = reinterpret_cast<float*>(VelocityData.data());

        UE_LOG(LogTemp, Display, TEXT("[ZMQController] Velocity array from Python: %f, %f, %f, %f"),
            VelocityArray[0], VelocityArray[1], VelocityArray[2], VelocityArray[3]);

        FVector DesiredVelocity(VelocityArray[0], VelocityArray[1], 0.f);
        DroneController->SetDesiredVelocity(DesiredVelocity);
        DroneController->SetHoverMode(true, VelocityArray[2]);
        DroneController->SetDesiredYawRate(VelocityArray[3]);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid velocity data size in ZMQ message."));
    }
}

void AZMQController::HandleObstacleCommand(zmq::multipart_t& Message)
{
    if (Message.empty()) {
        UE_LOG(LogTemp, Warning, TEXT("Empty obstacle message"));
        return;
    }

    zmq::message_t ObstacleCountMsg = Message.pop();
    int32 numObstacles = 0;
    
    if (ObstacleCountMsg.size() == sizeof(float)) {
        numObstacles = static_cast<int32>(*static_cast<float*>(ObstacleCountMsg.data()));
        UE_LOG(LogTemp, Display, TEXT("Obstacle count: %d"), numObstacles);
    } else {
        UE_LOG(LogTemp, Warning, TEXT("Invalid obstacle count data size: %d"), ObstacleCountMsg.size());
        return;
    }

    bool bRandomize = false;
    if (!Message.empty()) {
        zmq::message_t randomizeBoolMsg = Message.pop();
        if (randomizeBoolMsg.size() == sizeof(bool)) {
            bRandomize = *static_cast<bool*>(randomizeBoolMsg.data());
        }
    }

    UE_LOG(LogTemp, Display, TEXT("Obstacle command: Count=%d, Randomize=%s"), 
       numObstacles, bRandomize ? TEXT("true") : TEXT("false"));

    if (!ObstacleManagerInstance) {
        TArray<AActor*> FoundActors;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AObstacleManager::StaticClass(), FoundActors);
        
        if (FoundActors.Num() > 0) {
            ObstacleManagerInstance = Cast<AObstacleManager>(FoundActors[0]);
            UE_LOG(LogTemp, Display, TEXT("Found ObstacleManager: %s"), 
                *ObstacleManagerInstance->GetName());
        } else {
            UE_LOG(LogTemp, Warning, TEXT("No ObstacleManager found in level! Will try to spawn one."));
            
            // Try to spawn an ObstacleManager if none exists
            FActorSpawnParameters SpawnParams;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            ObstacleManagerInstance = GetWorld()->SpawnActor<AObstacleManager>(AObstacleManager::StaticClass(), 
                                                                               FVector::ZeroVector, 
                                                                               FRotator::ZeroRotator, 
                                                                               SpawnParams);
            if (ObstacleManagerInstance) {
                UE_LOG(LogTemp, Display, TEXT("Spawned new ObstacleManager"));
            } else {
                UE_LOG(LogTemp, Error, TEXT("Failed to spawn ObstacleManager"));
                return;
            }
        }
    }
    
    // Call the ObstacleManager to create obstacles
    EGoalPosition goalPos = bRandomize ? EGoalPosition::Random : EGoalPosition::Front;
    if (ObstacleManagerInstance) {
        ObstacleManagerInstance->CreateObstacles(numObstacles, goalPos);
        UE_LOG(LogTemp, Display, TEXT("Created %d obstacles via ObstacleManager"), numObstacles);
    }
}

void AZMQController::SendStateData()
{
    if (!ControlSocket || !DronePawn) return;

    UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(DronePawn->GetRootComponent());
    if (!RootPrimitive) return;

    FVector CurrentVelocity = DroneController->GetCurrentLocalVelocity();
    FVector CurrentPosition = DronePawn->GetActorLocation();
    FRotator CurrentRotation = DronePawn->GetActorRotation();

    if (ObstacleManagerInstance) {
        CurrentGoalPosition = ObstacleManagerInstance->GetGoalPosition();
    }
    
    // Retrieve collision state from the Pawn's helper function
    bool bCollision = DronePawn->HasCollided();
    UE_LOG(LogTemp, Display, TEXT("Drone collision state is: %s"), bCollision ? TEXT("Colliding") : TEXT("Not Colliding"));

    // Create state data including collision info
    FString StateData = FString::Printf(
        TEXT("VELOCITY:%f,%f,%f;POSITION:%f,%f,%f;GOAL:%f,%f,%f;ATTITUDE:%f,%f,%f;COLLISION:%d"),
        CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Z,
        CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
        CurrentGoalPosition.X, CurrentGoalPosition.Y, CurrentGoalPosition.Z,
        CurrentRotation.Roll, CurrentRotation.Pitch, CurrentRotation.Yaw,
        bCollision ? 1 : 0
    );
    
    try
    {
        zmq::multipart_t Message;
        Message.addstr(TCHAR_TO_UTF8(*StateData));
        Message.send(*ControlSocket, static_cast<int>(zmq::send_flags::none));
    }
    catch (const zmq::error_t& Error)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to send state data: %s"),
               *FString(UTF8_TO_TCHAR(Error.what())));
    }
}

void AZMQController::InitializeImageCapture()
{
    if (!DronePawn || !DronePawn->CameraFPV) return;

    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RenderTarget->InitCustomFormat(
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        PF_B8G8R8A8,
        false
    );
    RenderTarget->UpdateResourceImmediate(true);

    CaptureComponent = NewObject<USceneCaptureComponent2D>(this);
    CaptureComponent->SetupAttachment(DronePawn->CameraFPV);
    CaptureComponent->RegisterComponent();
    CaptureComponent->HiddenActors.Add(DronePawn);
    CaptureComponent->TextureTarget = RenderTarget;
    if (DronePawn->CameraFPV)
    {
        UE_LOG(LogTemp, Display, TEXT("CameraFPV is valid: %s"), *DronePawn->CameraFPV->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("CameraFPV is null!"));
    }

    CaptureComponent->bCaptureEveryFrame = false;
    CaptureComponent->bCaptureOnMovement = false;
    CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
    CaptureComponent->bAlwaysPersistRenderingState = true;

}

void AZMQController::ProcessImageCapture()
{
    if (bIsCapturing || !CaptureComponent || !RenderTarget)
    {
        return;
    }
    
    bIsCapturing = true;

    const FVector CaptureLocation = DronePawn->CameraFPV->GetComponentLocation();
    const FRotator CameraFullRotation = DronePawn->CameraFPV->GetComponentRotation();
    const FRotator CaptureYawOnlyRotation = FRotator(0.0f, CameraFullRotation.Yaw, 0.0f);
    CaptureComponent->SetWorldLocationAndRotation(CaptureLocation, CaptureYawOnlyRotation);

    CaptureComponent->CaptureScene();
    RenderTarget->UpdateResourceImmediate(false);
    
    FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    if (!RenderTargetResource)
    {
        bIsCapturing = false;
        return;
    }
    
    FIntRect Rect(0, 0, Configuration.ImageResolution.X, Configuration.ImageResolution.Y);
    
    TArray<FColor>* ImageDataPtr = new TArray<FColor>();
    
    ENQUEUE_RENDER_COMMAND(AsyncReadPixelsCommand)(
        [this, RenderTargetResource, Rect, ImageDataPtr](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.ReadSurfaceData(
                RenderTargetResource->GetRenderTargetTexture(),
                Rect,
                *ImageDataPtr,
                FReadSurfaceDataFlags(RCM_UNorm)
            );
            
            AsyncTask(ENamedThreads::GameThread, [this, ImageDataPtr]()
            {
                // UE_LOG(LogTemp, Display, TEXT("Captured image via render command: %d pixels"), ImageDataPtr->Num());
                
                AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this, ImageDataPtr]()
                {
                    TArray<uint8> CompressedData = CompressImageData(*ImageDataPtr);
                    
                    if (PublishSocket)
                    {
                        try
                        {
                            zmq::multipart_t Message;
                            // Message.addstr(TCHAR_TO_UTF8(*Configuration.DroneID));
                            Message.addmem(CompressedData.GetData(), CompressedData.Num());
                            Message.send(*PublishSocket, static_cast<int>(zmq::send_flags::none));
                        }
                        catch (const zmq::error_t& Error)
                        {
                            UE_LOG(LogTemp, Warning, TEXT("Failed to send image data: %s"),
                                   *FString(UTF8_TO_TCHAR(Error.what())));
                        }
                    }
                    
                    delete ImageDataPtr;
                    bIsCapturing = false;
                });
            });
        }
    );
}



TArray<uint8> AZMQController::CompressImageData(const TArray<FColor>& ImageData)
{
    TArray<FColor> ResizedData;
    FImageUtils::ImageResize(
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        ImageData,
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        ResizedData,
        false
    );

    TArray64<uint8> CompressedData;
    FImageUtils::PNGCompressImageArray(
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        ResizedData,
        CompressedData
    );

    return TArray<uint8>(CompressedData.GetData(), CompressedData.Num());
}

void AZMQController::SetConfiguration(const FZMQConfiguration& NewConfig)
{
    Configuration = NewConfig;

    if (Configuration.CaptureInterval > 0.0f)
    {
        GetWorld()->GetTimerManager().SetTimer(
            ImageCaptureTimerHandle,
            this,
            &AZMQController::ProcessImageCapture,
            Configuration.CaptureInterval,
            true
        );
    }
    else
    {
        GetWorld()->GetTimerManager().ClearTimer(ImageCaptureTimerHandle);
    }
}

void AZMQController::SetDroneID(const FString& NewID)
{
    Configuration.DroneID = NewID;
    UE_LOG(LogTemp, Display, TEXT("ZMQController DroneID set to: %s"), *Configuration.DroneID);
}

void AZMQController::CheckAndInitialize()
{
    if (TargetPawn && TargetPawn->QuadController)
    {
        DronePawn = TargetPawn;
        DroneController = TargetPawn->QuadController;
        Configuration.DroneID = TargetPawn->GetName();
        Initialize(DronePawn, DroneController, Configuration);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("ZMQController: Still no QuadDroneController found on TargetPawn after delay."));
    }
}