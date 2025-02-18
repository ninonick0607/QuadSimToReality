#pragma once
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "HAL/ThreadSafeBool.h"
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ZMQController.generated.h"

namespace zmq { class context_t; }

// Forward declarations
class AQuadPawn;
class UQuadDroneController;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class AActor;
enum class EFlightMode : uint8;

USTRUCT(BlueprintType)
struct FZMQConfiguration
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category = "ZMQ")
    int32 PublishPort = 5557;

    UPROPERTY(EditAnywhere, Category = "ZMQ")
    int32 CommandPort = 5556;

    UPROPERTY(EditAnywhere, Category = "ZMQ")
    int32 ControlPort = 5558;

    UPROPERTY(EditAnywhere, Category = "Image Capture")
    FIntPoint ImageResolution = FIntPoint(128, 128);

    UPROPERTY(EditAnywhere, Category = "Image Capture")
    float CaptureInterval = 0.0001f;

    UPROPERTY(EditAnywhere, Category = "Communication")
    FString DroneID = TEXT("drone1");
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UZMQController : public UActorComponent
{
    GENERATED_BODY()

public:
    UZMQController();
    virtual ~UZMQController();

    void Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FZMQConfiguration& Config);

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void SetConfiguration(const FZMQConfiguration& NewConfig);

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    const FZMQConfiguration& GetConfiguration() const { return Configuration; }

    FVector GetCurrentGoalPosition() const { return CurrentGoalPosition; }
    void SetDroneID(const FString& NewID);

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, 
                             FActorComponentTickFunction* ThisTickFunction) override;

private:
    void InitializeImageCapture();
    void ProcessImageCapture();
    void InitializeZMQ();
    void HandleResetCommand();
    void HandleVelocityCommand(zmq::multipart_t& Message);
    void SendStateData();
    void ProcessCommands();
    TArray<uint8> CompressImageData(const TArray<FColor>& ImageData);

    FRunnableThread* ZMQThread;
    FThreadSafeBool bRunZMQ;
    TQueue<TArray<uint8>, EQueueMode::Mpsc> ImageQueue;
    FCriticalSection CommandMutex;
    
    zmq::context_t Context;
    TSharedPtr<zmq::socket_t> PublishSocket;
    TSharedPtr<zmq::socket_t> CommandSocket;
    TSharedPtr<zmq::socket_t> ControlSocket;

    UPROPERTY(EditAnywhere, Category = "ZMQ")
    FZMQConfiguration Configuration;

    FTimerHandle ImageCaptureTimerHandle;
    TAtomic<bool> bIsCapturing;
    TAtomic<bool> bIsProcessingCommand;
    
    UPROPERTY()
    TObjectPtr<AQuadPawn> DronePawn;

    UPROPERTY()
    TObjectPtr<UQuadDroneController> DroneController;

    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> CaptureComponent;

    UPROPERTY()
    TObjectPtr<UTextureRenderTarget2D> RenderTarget;

    FVector CurrentGoalPosition;
};