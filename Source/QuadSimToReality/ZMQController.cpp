

        #include "ZMQController.h"
        #include <zmq.hpp>
        #include <zmq_addon.hpp>
        #include "Kismet/GameplayStatics.h"
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
            DroneID = TEXT("drone1"); 
            DronePawn = Cast<AQuadPawn>(GetOwner());
            if (DronePawn) {

                DroneController = NewObject<UQuadDroneController>(this, UQuadDroneController::StaticClass());
                DroneController->Initialize(DronePawn);
                UE_LOG(LogTemp, Display, TEXT("DroneController initialized for %s"), *DronePawn->GetName());

                InitialPosition = DronePawn->GetActorLocation();
            }
            else {
                UE_LOG(LogTemp, Error, TEXT("DronePawn not found!"));
                DroneController = nullptr;
            }

            TArray<AActor*> FoundActors;
            UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("goal"), FoundActors);

            if (FoundActors.Num() > 0) {
                SM_PillarFrameActor = FoundActors[0];
                UE_LOG(LogTemp, Display, TEXT("Found SM_PillarFrame with tag: %s"), *SM_PillarFrameActor->GetName());
            }
            else {
                UE_LOG(LogTemp, Warning, TEXT("No actors found with the tag 'goal'!"));
            }


            AQuadPawn* DroneActor = Cast<AQuadPawn>(GetOwner());
            if (DroneActor) {
                UE_LOG(LogTemp, Display, TEXT("DroneActor found: %s"), *DroneActor->GetName());

                if (DroneActor->CameraFPV) {  
                    if (!RenderTarget) {
                        RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());
                        RenderTarget->InitAutoFormat(128, 128);
                        RenderTarget->UpdateResourceImmediate(true);
                    }

                    if (!SceneCaptureComponent) {
                        SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D::StaticClass());
                        SceneCaptureComponent->SetupAttachment(DroneActor->CameraFPV); 
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

                    FString Address = TEXT("tcp://*:5557");
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
                    FString Address = TEXT("tcp://localhost:5556");  
                    CommandSocket->connect(TCHAR_TO_UTF8(*Address));
                    CommandSocket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
                    UE_LOG(LogTemp, Display, TEXT("ZeroMQ SUB Socket connected to %s for commands"), *Address);
                }
                catch (const zmq::error_t& e)
                {
                    UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during command socket setup: %s"), *FString(e.what()));
                }
            }

            if (!ControlSocket) {
                try {
                    ControlSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::pub);
                    FString ControlAddress = TEXT("tcp://*:5558");
                    ControlSocket->bind(TCHAR_TO_UTF8(*ControlAddress));
                    UE_LOG(LogTemp, Display, TEXT("ZeroMQ Control PUB Socket bound to %s"), *ControlAddress);
                }
                catch (const zmq::error_t& e) {
                    UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during control socket setup: %s"), *FString(e.what()));
                }
            }

            UE_LOG(LogTemp, Display, TEXT("BeginPlay() execution completed in UZMQController"));
        }

        void UZMQController::EndPlay(const EEndPlayReason::Type EndPlayReason) {
            bIsActive = false;

            if (CommandSocket) {
                CommandSocket->close();
                delete CommandSocket;
                CommandSocket = nullptr;
                UE_LOG(LogTemp, Display, TEXT("Command SUB Socket closed"));
            }

            if (ControlSocket) {
                ControlSocket->close();
                delete ControlSocket;
                ControlSocket = nullptr;
                UE_LOG(LogTemp, Display, TEXT("Control PUB Socket closed"));
            }


            SharedResourceRefCount--;

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
                UE_LOG(LogTemp, Warning, TEXT("Capture"));
                CaptureAndSendImage();
            }
            else {
                TimeSinceLastUpdate += DeltaTime;

                if (TimeSinceLastUpdate >= UpdateInterval)
                {
                    UE_LOG(LogTemp, Warning, TEXT("Capture U"));
                    CaptureAndSendImage();
                    TimeSinceLastUpdate = 0.0f; 
                }
            }
            ReceiveVelocityCommand();
            
            SendData();

        }

        

        void UZMQController::SendData()
        {
            if (!ControlSocket || !DronePawn) {
                UE_LOG(LogTemp, Warning, TEXT("ControlSocket or DronePawn is null, unable to send unified data."));
                return;
            }


            UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(DronePawn->GetRootComponent());
            FVector CurrentVelocity = RootPrimitive ? RootPrimitive->GetPhysicsLinearVelocity() : FVector::ZeroVector;

            FVector CurrentPosition = DronePawn->GetActorLocation();
            UE_LOG(LogTemp, Display, TEXT("Goal Position Before Sending: %s"), *GoalPosition.ToString());

            FString UnifiedData = FString::Printf(
                TEXT("VELOCITY:%f,%f,%f;POSITION:%f,%f,%f;GOAL:%f,%f,%f"),
                CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Z,
                CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
                GoalPosition.X, GoalPosition.Y, GoalPosition.Z
            );


            zmq::multipart_t multipart;
            multipart.addstr(TCHAR_TO_UTF8(*UnifiedData));

            try {
                multipart.send(*ControlSocket, static_cast<int>(zmq::send_flags::none));
                UE_LOG(LogTemp, Display, TEXT("Sent Unified Data: %s"), *UnifiedData);
            }
            catch (const zmq::error_t& e) {
                UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during sending unified data: %s"), UTF8_TO_TCHAR(e.what()));
            }
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
                    if (multipart.empty())
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Received empty multipart message"));
                        return;
                    }

                    std::string topicOrCommand = multipart.popstr();


                    if (topicOrCommand == "RESET")
                    {
                        if (DronePawn)
                        {


                            FVector NewPosition = FVector(
                                FMath::RandRange(-500.0f, 500.0f), 
                                FMath::RandRange(-500.0f, 500.0f), 
                                InitialPosition.Z              
                            );

                            FHitResult HitResult;

                            if (SM_PillarFrameActor) {
                                SM_PillarFrameActor->SetActorLocation(NewPosition, false, &HitResult, ETeleportType::TeleportPhysics);
                                UE_LOG(LogTemp, Display, TEXT("SM_PillarFrameActor position set to %s"), *NewPosition.ToString());
                            }
                            else {
                                UE_LOG(LogTemp, Warning, TEXT("SM_PillarFrameActor is null, unable to set position."));
                            }

                            GoalPosition = NewPosition;

                            DronePawn->SetActorLocation(InitialPosition, false, &HitResult, ETeleportType::TeleportPhysics);
                            DronePawn->SetActorRotation(FRotator::ZeroRotator);

                            UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(DronePawn->GetRootComponent());
                            if (RootPrimitive)
                            {
                                RootPrimitive->SetPhysicsLinearVelocity(FVector::ZeroVector);
                                RootPrimitive->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                            }

                            UE_LOG(LogTemp, Display, TEXT("Drone position has been reset to %s"), *InitialPosition.ToString());
                        }
                        else
                        {
                            UE_LOG(LogTemp, Warning, TEXT("DronePawn is null"));
                        }
                    }
                    else if (topicOrCommand == "VELOCITY")
                    {
                        if (multipart.empty())
                        {
                            UE_LOG(LogTemp, Warning, TEXT("No message part after topic"));
                            return;
                        }

                        zmq::message_t message = multipart.pop();

                        if (message.size() == sizeof(float) * 3)
                        {
                            if (message.data())
                            {
                                float* velocityArray = reinterpret_cast<float*>(message.data());
                                FVector DesiredVelocity(velocityArray[0], velocityArray[1], velocityArray[2]);

                                if (DroneController)
                                {
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
                        UE_LOG(LogTemp, Warning, TEXT("Unknown command received: %s"), *FString(topicOrCommand.c_str()));
                    }
                }
                else
                {

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
                UE_LOG(LogTemp, Warning, TEXT("1"));
                return;
            }

            FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
            if (!RenderTargetResource)
            {
                UE_LOG(LogTemp, Warning, TEXT("2"));
                return;
            }

            TArray<FColor> Bitmap;
            if (!RenderTargetResource->ReadPixels(Bitmap))
            {
                UE_LOG(LogTemp, Warning, TEXT("3"));
                return;
            }

            int32 NewSizeX = RenderTarget->SizeX;
            int32 NewSizeY = RenderTarget->SizeY;
            TArray<FColor> ResizedBitmap;
            FImageUtils::ImageResize(RenderTarget->SizeX, RenderTarget->SizeY, Bitmap, NewSizeX, NewSizeY, ResizedBitmap, false);
            UE_LOG(LogTemp, Warning, TEXT("RenderTarget Size: %d x %d"), RenderTarget->SizeX, RenderTarget->SizeY);
            UE_LOG(LogTemp, Warning, TEXT("4"));
            UE_LOG(LogTemp, Warning, TEXT("ResizedBitmap Size: %d"), ResizedBitmap.Num());
            TArray64<uint8> CompressedBitmap;
            FImageUtils::PNGCompressImageArray(NewSizeX, NewSizeY, ResizedBitmap, CompressedBitmap);

            TArray<uint8> CompressedBitmapArray(CompressedBitmap.GetData(), CompressedBitmap.Num());
            UE_LOG(LogTemp, Warning, TEXT("CompressedBitmap Size: %d"), CompressedBitmap.Num());

            SendImageData(CompressedBitmapArray);
        }

        void UZMQController::SendImageData(const TArray<uint8>&CompressedBitmap)
        {
            if (!ZMQSocket)
            {
                UE_LOG(LogTemp, Warning, TEXT("5"));
                return;
            }
            UE_LOG(LogTemp, Warning, TEXT("6"));
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