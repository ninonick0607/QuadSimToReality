

        // Initialize DroneController with the drone pawn

        #include "ZMQController.h"
        #include <zmq.hpp>
        #include <zmq_addon.hpp>
        #include "Engine/TextureRenderTarget2D.h"
        #include "ImageUtils.h"
        #include "QuadSimToReality/QuadPawn.h"
        #include "QuadSimToReality/QuadDroneController.h"
        #include "Camera/CameraComponent.h"
        #include "Async/Async.h"
        #include "Containers/ArrayView.h"

        int32 UZMQController::SharedResourceRefCount = 0;
        zmq::context_t* UZMQController::SharedZMQContext = nullptr;
        zmq::socket_t* UZMQController::SharedZMQSocket = nullptr;

        UZMQController::UZMQController()
            : UpdateInterval(0.0001f), TimeSinceLastUpdate(0.0f), bIsActive(true) {
            PrimaryComponentTick.bCanEverTick = true;
        }

        void UZMQController::BeginPlay() {
            Super::BeginPlay();
            SharedResourceRefCount++;
            UE_LOG(LogTemp, Display, TEXT("BeginPlay() called in UZMQController"));

            AQuadPawn* DronePawn = Cast<AQuadPawn>(GetOwner());
            if (DronePawn) {
                // Initialize DroneController with the drone pawn
                DroneController = NewObject<UQuadDroneController>(this, UQuadDroneController::StaticClass());
                DroneController->Initialize(DronePawn);
                UE_LOG(LogTemp, Display, TEXT("DroneController initialized for %s"), *DronePawn->GetName());
            }
            else {
                UE_LOG(LogTemp, Error, TEXT("DronePawn not found!"));
                DroneController = nullptr;
            }

            AQuadPawn* DroneActor = Cast<AQuadPawn>(GetOwner());
            if (DroneActor) {
                UE_LOG(LogTemp, Display, TEXT("DroneActor found: %s"), *DroneActor->GetName());

                if (DroneActor->CameraFPV) {  // Use CameraFPV as specified
                    if (!RenderTarget) {
                        RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());
                        RenderTarget->InitAutoFormat(128, 128);
                        RenderTarget->UpdateResourceImmediate(true);
                    }

                    if (!SceneCaptureComponent) {
                        SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D::StaticClass());
                        SceneCaptureComponent->SetupAttachment(DroneActor->CameraFPV);  // Attach to the CameraFPV component
                        SceneCaptureComponent->RegisterComponent();
                    }

                    SceneCaptureComponent->TextureTarget = RenderTarget;
                    UE_LOG(LogTemp, Display, TEXT("SceneCaptureComponent attached to CameraFPV"));
                }
                else {
                    UE_LOG(LogTemp, Warning, TEXT("CameraFPV component not found on DroneActor!"));
                }
            }
            else {
                UE_LOG(LogTemp, Warning, TEXT("DroneActor not found!"));
            }

            if (!SharedZMQContext) {
                SharedZMQContext = new zmq::context_t(1);
            }


            if (!SharedZMQSocket) {
                try {
                    SharedZMQSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::pub);

                    FString Address = TEXT("tcp://*:5555");
                    SharedZMQSocket->bind(TCHAR_TO_UTF8(*Address));
                    UE_LOG(LogTemp, Display, TEXT("ZeroMQ PUB Socket bound to %s"), *Address);
                }
                catch (const zmq::error_t& e) {
                    UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during socket setup: %s"), *FString(e.what()));
                }
            }

            ZMQSocket = SharedZMQSocket;

            if (!CommandSocket)
            {
                try
                {
                    CommandSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::sub);
                    FString Address = TEXT("tcp://localhost:5556");  // Use the same port as in Python
                    CommandSocket->connect(TCHAR_TO_UTF8(*Address));
                    CommandSocket->setsockopt(ZMQ_SUBSCRIBE, "", 0);  // Subscribe to all topics
                    UE_LOG(LogTemp, Display, TEXT("ZeroMQ SUB Socket connected to %s for commands"), *Address);
                }
                catch (const zmq::error_t& e)
                {
                    UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during command socket setup: %s"), *FString(e.what()));
                }
            }


            UE_LOG(LogTemp, Display, TEXT("BeginPlay() execution completed in UZMQController"));
        }

        void UZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason) {
            bIsActive = false; // Stop TickComponent execution

            // Close individual resources
            if (CommandSocket) {
                CommandSocket->close();
                delete CommandSocket;
                CommandSocket = nullptr;
                UE_LOG(LogTemp, Display, TEXT("Command SUB Socket closed"));
            }

        /*    if (DroneController) {
                delete DroneController;
                DroneController = nullptr;
            }*/

            // Decrement shared resource reference count
            SharedResourceRefCount--;

            // Cleanup shared resources only if no references remain
            if (SharedResourceRefCount == 0) {
                if (SharedZMQSocket) {
                    SharedZMQSocket->close();
                    delete SharedZMQSocket;
                    SharedZMQSocket = nullptr;
                    UE_LOG(LogTemp, Display, TEXT("Shared PUB Socket closed"));
                }

                if (SharedZMQContext) {
                    SharedZMQContext->close();
                    delete SharedZMQContext;
                    SharedZMQContext = nullptr;
                    UE_LOG(LogTemp, Display, TEXT("Shared ZMQ Context closed"));
                }
            }

            Super::EndPlay(EndPlayReason);
            UE_LOG(LogTemp, Display, TEXT("EndPlay() completed"));
        }
        void UZMQController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction * ThisTickFunction)
        {

            if (!bIsActive) return;
            Super::TickComponent(DeltaTime, TickType, ThisTickFunction);



            if (UpdateInterval == 0.0) {
                CaptureAndSendImage();
            }
            else {
                TimeSinceLastUpdate += DeltaTime;

                // Check if it's time to capture and send a new image
                if (TimeSinceLastUpdate >= UpdateInterval)
                {
                    CaptureAndSendImage();
                    TimeSinceLastUpdate = 0.0f; // Reset the timer
                }
            }
            ReceiveVelocityCommand();
        }

        void UZMQController::ReceiveVelocityCommand()
        {
            if (!CommandSocket)
            {
                UE_LOG(LogTemp, Warning, TEXT("CommandSocket is null"));
                return;
            }

            zmq::multipart_t multipart;
            try {
                bool result = multipart.recv(*CommandSocket, static_cast<int>(zmq::recv_flags::dontwait));

                if (result)
                {
                    // Message received
                    if (multipart.empty())
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Received empty multipart message"));
                        return;
                    }

                    // Extract topic
                    std::string topic = multipart.popstr();

                    if (multipart.empty())
                    {
                        UE_LOG(LogTemp, Warning, TEXT("No message part after topic"));
                        return;
                    }

                    // Extract message
                    zmq::message_t message = multipart.pop();

                    // Ensure the message size is correct
                    if (message.size() == sizeof(float) * 3)
                    {
                        // Ensure message.data() is not null
                        if (message.data())
                        {
                            float* velocityArray = reinterpret_cast<float*>(message.data());
                            FVector DesiredVelocity(velocityArray[0], velocityArray[1], velocityArray[2]);

                            if (DroneController)
                            {
                                // Instead of directly setting desiredNewVelocity, call MoveByVelocity
                                DroneController->SetDesiredVelocity(DesiredVelocity);
                                DroneController->SetFlightMode(UQuadDroneController::FlightMode::VelocityControl);
                                UE_LOG(LogTemp, Display, TEXT("Received velocity command: %s"), *DesiredVelocity.ToString());
                            }
                        }
                        else
                        {
                            UE_LOG(LogTemp, Warning, TEXT("Received message data is null"));
                        }
                    }
                    else
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Received invalid velocity command size: %zu bytes"), message.size());
                    }
                }
                else
                {
                    // No message received; this is expected in non-blocking mode
                    // You can optionally log this or simply continue
                }

            }
            catch (const zmq::error_t& e) {
                UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during receive: %s"), UTF8_TO_TCHAR(e.what()));
            }


        }

        void UZMQController::CaptureAndSendImage()
        {
            if (!SceneCaptureComponent || !RenderTarget)
            {
                return;
            }

            FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
            if (!RenderTargetResource)
            {
                return;
            }

            TArray<FColor> Bitmap;
            if (!RenderTargetResource->ReadPixels(Bitmap))
            {
                return;
            }

            int32 NewSizeX = RenderTarget->SizeX;
            int32 NewSizeY = RenderTarget->SizeY;
            TArray<FColor> ResizedBitmap;
            FImageUtils::ImageResize(RenderTarget->SizeX, RenderTarget->SizeY, Bitmap, NewSizeX, NewSizeY, ResizedBitmap, false);

            TArray64<uint8> CompressedBitmap;
            FImageUtils::PNGCompressImageArray(NewSizeX, NewSizeY, ResizedBitmap, CompressedBitmap);

            TArray<uint8> CompressedBitmapArray(CompressedBitmap.GetData(), CompressedBitmap.Num());

            SendImageData(CompressedBitmapArray);
        }

        void UZMQController::SendImageData(const TArray<uint8>&CompressedBitmap)
        {
            if (!ZMQSocket)
            {
                return;
            }

            zmq::multipart_t multipart;
            multipart.addstr(TCHAR_TO_UTF8(*DroneID));
            multipart.addmem(CompressedBitmap.GetData(), CompressedBitmap.Num());

            try
            {
                multipart.send(*ZMQSocket, static_cast<int>(zmq::send_flags::none));
                UE_LOG(LogTemp, Verbose, TEXT("Sent image data from drone %s of size: %d bytes"), *DroneID, CompressedBitmap.Num());
            }
            catch (const zmq::error_t& e)
            {
                UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during sending image data: %s"), UTF8_TO_TCHAR(e.what()));
            }
        }