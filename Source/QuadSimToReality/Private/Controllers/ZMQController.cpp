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

AZMQController::AZMQController()
    : bIsCapturing(false)
    , bIsProcessingCommand(false)
    , DronePawn(nullptr)
    , DroneController(nullptr)
    , CaptureComponent(nullptr)
    , RenderTarget(nullptr)
    , CurrentGoalPosition(FVector(0.0f, 0.0f, 1000.0f))
{
    PrimaryActorTick.bCanEverTick = true;
}

AZMQController::~AZMQController()
{
}

void AZMQController::Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FZMQConfiguration& Config)
{
    UE_LOG(LogTemp, Display, TEXT("UZMQController::Initialize called with Pawn=%s, Controller=%s"),
       InPawn ? *InPawn->GetName() : TEXT("nullptr"),
       InDroneController ? *InDroneController->GetName() : TEXT("nullptr"));    

    Configuration = Config;
    DronePawn = InPawn;
    DroneController = InDroneController;
    UE_LOG(LogTemp, Display, TEXT("UZMQController initialized with DroneID: %s"), *Configuration.DroneID);

    InitializeZMQ();
    InitializeImageCapture();

}

void AZMQController::BeginPlay()
{
    Super::BeginPlay();
    
    // If TargetPawn is set in Blueprint, use it:
    if (TargetPawn && !DronePawn)
    {
        DronePawn = TargetPawn;
        DroneController = DronePawn->QuadController;
        if (DroneController)
        {
            Initialize(DronePawn, DroneController, Configuration);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("AZMQController: No QuadDroneController found on TargetPawn."));
        }
    }

    UE_LOG(LogTemp, Display, TEXT("Initial goal position set to: X=%f, Y=%f, Z=%f"),
           CurrentGoalPosition.X, CurrentGoalPosition.Y, CurrentGoalPosition.Z);

    // Start image capture timer
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
}

void AZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorld()->GetTimerManager().ClearTimer(ImageCaptureTimerHandle);
    PublishSocket.Reset();
    CommandSocket.Reset();
    ControlSocket.Reset();
    Super::EndPlay(EndPlayReason);
}

void AZMQController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Process incoming commands
    ProcessCommands();
    // Send state data
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
    if (!CommandSocket || bIsProcessingCommand) return;

    bIsProcessingCommand = true;

    try
    {
        zmq::multipart_t Message;
        if (Message.recv(*CommandSocket, static_cast<int>(zmq::recv_flags::dontwait)))
        {
            if (!Message.empty())
            {
                std::string Command = Message.popstr();

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
                    HandleVelocityCommand(Message);
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
    CurrentGoalPosition = FVector(0.0f, 0.0f, FMath::RandRange(500.0f, 1500.0f));

    UE_LOG(LogTemp, Warning, TEXT("AZMQController: New goal height set to: Z=%f"), CurrentGoalPosition.Z);
    DroneController->ResetDroneOrigin();
}

void AZMQController::HandleVelocityCommand(zmq::multipart_t& Message)
{
    if (!DroneController || Message.empty()) return;

    zmq::message_t VelocityData = Message.pop();
    if (VelocityData.size() == sizeof(float) * 3 && VelocityData.data())
    {
        float* VelocityArray = reinterpret_cast<float*>(VelocityData.data());

        // Add a log so you know exactly what values you got
        UE_LOG(LogTemp, Display, TEXT("[ZMQController] Velocity array from Python: %f, %f, %f"),
            VelocityArray[0], VelocityArray[1], VelocityArray[2]);

        FVector DesiredVelocity(VelocityArray[0], VelocityArray[1], VelocityArray[2]);
        DroneController->SetDesiredVelocity(DesiredVelocity);
        DroneController->SetFlightMode(EFlightMode::VelocityControl);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Invalid velocity data size in ZMQ message."));
    }
}


void AZMQController::SendStateData()
{
    if (!ControlSocket || !DronePawn) return;

    UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(DronePawn->GetRootComponent());
    if (!RootPrimitive) return;

    FVector CurrentVelocity = RootPrimitive->GetPhysicsLinearVelocity();
    FVector CurrentPosition = DronePawn->GetActorLocation();

    try
    {
        FString StateData = FString::Printf(
            TEXT("VELOCITY:%f,%f,%f;POSITION:%f,%f,%f;GOAL:%f,%f,%f"),
            CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Z,
            CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
            CurrentGoalPosition.X, CurrentGoalPosition.Y, CurrentGoalPosition.Z
        );

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

    // Create render target with specified resolution
    RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RenderTarget->InitCustomFormat(
        Configuration.ImageResolution.X,
        Configuration.ImageResolution.Y,
        PF_B8G8R8A8,
        false
    );
    RenderTarget->UpdateResourceImmediate(true);

    // Create and setup capture component
    CaptureComponent = NewObject<USceneCaptureComponent2D>(this);
    CaptureComponent->SetupAttachment(DronePawn->CameraFPV);
    CaptureComponent->RegisterComponent();
    CaptureComponent->TextureTarget = RenderTarget;
    
    // Optimize capture settings
    CaptureComponent->bCaptureEveryFrame = false;
    CaptureComponent->bCaptureOnMovement = false;
    CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
}
void AZMQController::ProcessImageCapture()
{
    if (bIsCapturing || !CaptureComponent || !RenderTarget) return;
    
    bIsCapturing = true;
    
    // Capture frame
    CaptureComponent->CaptureScene();
    
    // Get render target resource
    FTextureRenderTargetResource* RenderTargetResource = 
        RenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        bIsCapturing = false;
        return;
    }

    // Read pixels on game thread
    TArray<FColor> ImageData;
    if (!RenderTargetResource->ReadPixels(ImageData))
    {
        bIsCapturing = false;
        return;
    }
    
    // Move the compression and sending to background thread
    AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this, ImageData]()
    {
        // Compress image data
        TArray<uint8> CompressedData = CompressImageData(ImageData);
        
        // Send compressed data
        if (PublishSocket)
        {
            try
            {
                zmq::multipart_t Message;
                Message.addstr(TCHAR_TO_UTF8(*Configuration.DroneID));
                Message.addmem(CompressedData.GetData(), CompressedData.Num());
                Message.send(*PublishSocket, static_cast<int>(zmq::send_flags::none));
            }
            catch (const zmq::error_t& Error)
            {
                UE_LOG(LogTemp, Warning, TEXT("Failed to send image data: %s"),
                       *FString(UTF8_TO_TCHAR(Error.what())));
            }
        }
        
        bIsCapturing = false;
    });
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
    // Update configuration
    Configuration = NewConfig;

    // Update image capture timer if needed
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
