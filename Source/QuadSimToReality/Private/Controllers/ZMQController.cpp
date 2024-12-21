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

// Static member initialization
zmq::context_t* UZMQController::SharedZMQContext = nullptr;
zmq::socket_t* UZMQController::SharedZMQSocket = nullptr;
int32 UZMQController::SharedResourceRefCount = 0;

UZMQController::UZMQController()
    : DroneID(TEXT("drone1"))
    , UpdateInterval(DEFAULT_UPDATE_INTERVAL)
    , TimeSinceLastUpdate(0.0f)
    , bIsActive(true)
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UZMQController::Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController)
{
    DronePawn = InPawn;
    DroneController = InDroneController;

    if (DronePawn)
    {
        InitialPosition = DronePawn->GetActorLocation();
        SetupImageCapture();
    }
}

void UZMQController::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Display, TEXT("ZMQController: Beginning initialization..."));

    SharedResourceRefCount++;
    UE_LOG(LogTemp, Display, TEXT("ZMQController: Resource count increased to %d"), SharedResourceRefCount);

    if (!DronePawn)
    {
        DronePawn = Cast<AQuadPawn>(GetOwner());
        if (DronePawn)
        {
            UE_LOG(LogTemp, Display, TEXT("ZMQController: Successfully found DronePawn: %s"), *DronePawn->GetName());
            InitialPosition = DronePawn->GetActorLocation();
            UE_LOG(LogTemp, Display, TEXT("ZMQController: Stored initial position: %s"), *InitialPosition.ToString());
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("ZMQController: Failed to find DronePawn!"));
        }
    }

    // Find goal actor
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("goal"), FoundActors);
    if (FoundActors.Num() > 0)
    {
        SM_PillarFrameActor = FoundActors[0];
        UE_LOG(LogTemp, Display, TEXT("ZMQController: Found goal actor: %s"), *SM_PillarFrameActor->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("ZMQController: No actors found with 'goal' tag!"));
    }

    InitializeZMQ();

    UE_LOG(LogTemp, Display, TEXT("ZMQController: Initialization completed"));
}

void UZMQController::InitializeZMQ()
{
    // Initialize shared context if needed
    if (!SharedZMQContext)
    {
        SharedZMQContext = new zmq::context_t(1);
    }

    // Initialize shared PUB socket
    if (!SharedZMQSocket)
    {
        try
        {
            SharedZMQSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::pub);
            SharedZMQSocket->bind("tcp://*:5557");
        }
        catch (const zmq::error_t& e)
        {
            UE_LOG(LogTemp, Error, TEXT("ZMQ PUB socket setup error: %s"), UTF8_TO_TCHAR(e.what()));
        }
    }
    ZMQSocket = SharedZMQSocket;

    // Initialize command socket (SUB)
    try
    {
        CommandSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::sub);
        CommandSocket->connect("tcp://localhost:5556");
        CommandSocket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }
    catch (const zmq::error_t& e)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ SUB socket setup error: %s"), UTF8_TO_TCHAR(e.what()));
    }

    // Initialize control socket
    try
    {
        ControlSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::pub);
        ControlSocket->bind("tcp://*:5558");
    }
    catch (const zmq::error_t& e)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Control socket setup error: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void UZMQController::SetupImageCapture()
{
    if (!DronePawn || !DronePawn->CameraFPV) return;

    // Setup render target
    if (!RenderTarget)
    {
        RenderTarget = NewObject<UTextureRenderTarget2D>(this);
        RenderTarget->InitAutoFormat(128, 128);
        RenderTarget->UpdateResourceImmediate(true);
    }

    // Setup capture component
    if (!SceneCaptureComponent)
    {
        SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this);
        SceneCaptureComponent->SetupAttachment(DronePawn->CameraFPV);
        SceneCaptureComponent->RegisterComponent();
        SceneCaptureComponent->TextureTarget = RenderTarget;
    }
}

void UZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    CleanupZMQ();
    Super::EndPlay(EndPlayReason);
}

void UZMQController::CleanupZMQ()
{
    bIsActive = false;

    // Cleanup individual sockets
    if (CommandSocket)
    {
        CommandSocket->close();
        delete CommandSocket;
        CommandSocket = nullptr;
    }

    if (ControlSocket)
    {
        ControlSocket->close();
        delete ControlSocket;
        ControlSocket = nullptr;
    }

    // Cleanup shared resources if last instance
    SharedResourceRefCount--;
    if (SharedResourceRefCount == 0)
    {
        if (SharedZMQSocket)
        {
            SharedZMQSocket->close();
            delete SharedZMQSocket;
            SharedZMQSocket = nullptr;
        }

        if (SharedZMQContext)
        {
            SharedZMQContext->close();
            delete SharedZMQContext;
            SharedZMQContext = nullptr;
        }
    }
}

void UZMQController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    if (!bIsActive) return;

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Handle image capture timing
    if (UpdateInterval == 0.0)
    {
        CaptureAndSendImage();
    }
    else
    {
        TimeSinceLastUpdate += DeltaTime;
        if (TimeSinceLastUpdate >= UpdateInterval)
        {
            CaptureAndSendImage();
            TimeSinceLastUpdate = 0.0f;
        }
    }

    // Process messages and send data
    ReceiveVelocityCommand();
    SendData();
}

void UZMQController::SendData()
{
    if (!ControlSocket || !DronePawn) return;

    // Get current velocity and position
    UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(DronePawn->GetRootComponent());
    FVector CurrentVelocity = RootPrimitive ? RootPrimitive->GetPhysicsLinearVelocity() : FVector::ZeroVector;
    FVector CurrentPosition = DronePawn->GetActorLocation();

    // Format unified data string
    FString UnifiedData = FString::Printf(
        TEXT("VELOCITY:%f,%f,%f;POSITION:%f,%f,%f;GOAL:%f,%f,%f"),
        CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Z,
        CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
        GoalPosition.X, GoalPosition.Y, GoalPosition.Z
    );

    // Send data
    try
    {
        zmq::multipart_t multipart;
        multipart.addstr(TCHAR_TO_UTF8(*UnifiedData));
        multipart.send(*ControlSocket, static_cast<int>(zmq::send_flags::none));
    }
    catch (const zmq::error_t& e)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Error sending unified data: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void UZMQController::ReceiveVelocityCommand()
{
    if (!CommandSocket) return;

    try
    {
        zmq::multipart_t multipart;
        if (multipart.recv(*CommandSocket, static_cast<int>(zmq::recv_flags::dontwait)))
        {
            if (multipart.empty()) return;

            std::string command = multipart.popstr();

            if (command == "RESET")
            {
                HandleResetCommand();
            }
            if (command == "INTEGRAL RESET")
            {
                UE_LOG(LogTemp, Error, TEXT("INTEGRAL RESET COMMAND RECEIVED"));
                DroneController->ResetVelocityDroneIntegral();

            }
            else if (command == "VELOCITY")
            {
                HandleVelocityCommand(multipart);
            }
        }
    }
    catch (const zmq::error_t& e)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Error receiving command: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void UZMQController::HandleResetCommand()
{
    if (!DronePawn || !DroneController) return;

    // Generate new random position for goal
    FVector NewPosition(
        FMath::RandRange(-500.0f, 500.0f),
        FMath::RandRange(-500.0f, 500.0f),
        InitialPosition.Z
    );

    // Update goal actor position
    if (SM_PillarFrameActor)
    {
        SM_PillarFrameActor->SetActorLocation(NewPosition, false, nullptr, ETeleportType::TeleportPhysics);
        GoalPosition = NewPosition;
    }

    // Use the existing reset function that handles all the physics properly
    DroneController->ResetDroneOrigin();
}

void UZMQController::HandleVelocityCommand(zmq::multipart_t& multipart)
{
    if (multipart.empty() || !DroneController) return;

    zmq::message_t message = multipart.pop();
    if (message.size() == sizeof(float) * 3 && message.data())
    {
        float* velocityArray = reinterpret_cast<float*>(message.data());
        FVector DesiredVelocity(velocityArray[0], velocityArray[1], velocityArray[2]);

        DroneController->SetDesiredVelocity(DesiredVelocity);
        DroneController->SetFlightMode(UQuadDroneController::FlightMode::VelocityControl);
    }
}

void UZMQController::CaptureAndSendImage()
{
    if (!SceneCaptureComponent || !RenderTarget) return;

    FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    if (!RenderTargetResource) return;

    TArray<FColor> Bitmap;
    if (!RenderTargetResource->ReadPixels(Bitmap)) return;

    // Resize and compress image
    TArray<FColor> ResizedBitmap;
    FImageUtils::ImageResize(RenderTarget->SizeX, RenderTarget->SizeY, Bitmap,
        RenderTarget->SizeX, RenderTarget->SizeY, ResizedBitmap, false);

    TArray64<uint8> CompressedBitmap;
    FImageUtils::PNGCompressImageArray(RenderTarget->SizeX, RenderTarget->SizeY,
        ResizedBitmap, CompressedBitmap);

    SendImageData(TArray<uint8>(CompressedBitmap.GetData(), CompressedBitmap.Num()));
}

void UZMQController::SendImageData(const TArray<uint8>& CompressedBitmap)
{
    if (!ZMQSocket) return;

    try
    {
        zmq::multipart_t multipart;
        multipart.addstr(TCHAR_TO_UTF8(*DroneID));
        multipart.addmem(CompressedBitmap.GetData(), CompressedBitmap.Num());
        multipart.send(*ZMQSocket, static_cast<int>(zmq::send_flags::none));
    }
    catch (const zmq::error_t& e)
    {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Error sending image: %s"), UTF8_TO_TCHAR(e.what()));
    }
}
