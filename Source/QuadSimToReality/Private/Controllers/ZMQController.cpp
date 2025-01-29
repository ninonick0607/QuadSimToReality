#include "Controllers/ZMQController.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "Kismet/GameplayStatics.h"
#include "ImageUtils.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "Camera/CameraComponent.h"
#include "HAL/RunnableThread.h"
#include "Async/Async.h"

UZMQController::UZMQController()
    : bIsCapturing(false)
    , bIsProcessingCommand(false)
    , ZMQThread(nullptr)
    , bRunZMQ(false)
    , DronePawn(nullptr)
    , DroneController(nullptr)
    , CaptureComponent(nullptr)
    , RenderTarget(nullptr)
    , PublishSocket(nullptr)
    , CommandSocket(nullptr)
    , ControlSocket(nullptr)
    , CurrentGoalPosition(FVector(0.0f, 0.0f, 1000.0f))
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickInterval = 0.0f;
}

UZMQController::~UZMQController()
{
    // Cleanup handled in EndPlay
}

void UZMQController::Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FZMQConfiguration& Config)
{
    Configuration = Config;
    DronePawn = InPawn;
    DroneController = InDroneController;
    InitializeZMQ();
}

void UZMQController::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Display, TEXT("Initial goal position set to: X=%f, Y=%f, Z=%f"), 
           CurrentGoalPosition.X, CurrentGoalPosition.Y, CurrentGoalPosition.Z);

    // // Start image capture timer
    // if (Configuration.CaptureInterval > 0.0f)
    // {
    //     GetWorld()->GetTimerManager().SetTimer(
    //         ImageCaptureTimerHandle,
    //         this,
    //         &UZMQController::ProcessImageCapture,
    //         Configuration.CaptureInterval,
    //         true
    //     );
    // }
}

void UZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorld()->GetTimerManager().ClearTimer(ImageCaptureTimerHandle);
    PublishSocket.Reset();
    CommandSocket.Reset();
    ControlSocket.Reset();
    Super::EndPlay(EndPlayReason);
}

void UZMQController::InitializeZMQ()
{
    try 
    {
        // Initialize publish socket
        PublishSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::pub);
        FString PublishEndpoint = FString::Printf(TEXT("tcp://*:%d"), Configuration.PublishPort);
        PublishSocket->bind(TCHAR_TO_UTF8(*PublishEndpoint));

        // Initialize command socket
        CommandSocket = MakeShared<zmq::socket_t>(Context, zmq::socket_type::sub);
        FString CommandEndpoint = FString::Printf(TEXT("tcp://localhost:%d"), Configuration.CommandPort);
        CommandSocket->connect(TCHAR_TO_UTF8(*CommandEndpoint));
        CommandSocket->set(zmq::sockopt::subscribe, "");

        // Initialize control socket
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



void UZMQController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Process incoming commands
    ProcessCommands();

    // Send state data
    SendStateData();
}

void UZMQController::ProcessCommands()
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

void UZMQController::HandleResetCommand()
{
    if (!DroneController) return;

    // Set new goal position (only varying Z height)
    CurrentGoalPosition = FVector(
        0.0f,  // Keep X at 0
        0.0f,  // Keep Y at 0
        FMath::RandRange(500.0f, 1500.0f)  // Random high Z goal
    );

    UE_LOG(LogTemp, Warning, TEXT("New goal height set to: Z=%f"), CurrentGoalPosition.Z);

    // Reset drone
    DroneController->ResetDroneOrigin();
}

void UZMQController::HandleVelocityCommand(zmq::multipart_t& Message)
{
    if (!DroneController || Message.empty()) return;

    zmq::message_t VelocityData = Message.pop();
    if (VelocityData.size() == sizeof(float) * 3 && VelocityData.data())
    {
        float* VelocityArray = reinterpret_cast<float*>(VelocityData.data());
        FVector DesiredVelocity(VelocityArray[0], VelocityArray[1], VelocityArray[2]);

        DroneController->SetDesiredVelocity(DesiredVelocity);
        DroneController->SetFlightMode(EFlightMode::VelocityControl);
    }
}

void UZMQController::SendStateData()
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

void UZMQController::InitializeImageCapture()
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
void UZMQController::ProcessImageCapture()
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

TArray<uint8> UZMQController::CompressImageData(const TArray<FColor>& ImageData)
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


void UZMQController::SetConfiguration(const FZMQConfiguration& NewConfig)
{
    // Update configuration
    Configuration = NewConfig;

    // Update image capture timer if needed
    if (Configuration.CaptureInterval > 0.0f)
    {
        GetWorld()->GetTimerManager().SetTimer(
            ImageCaptureTimerHandle,
            this,
            &UZMQController::ProcessImageCapture,
            Configuration.CaptureInterval,
            true
        );
    }
    else
    {
        GetWorld()->GetTimerManager().ClearTimer(ImageCaptureTimerHandle);
    }
}