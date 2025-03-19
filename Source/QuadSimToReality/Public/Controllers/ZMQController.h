#pragma once

#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
class SZMQImageWidget;

#include "ZMQController.generated.h"
// Forward declarations
class AQuadPawn;
class UQuadDroneController;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

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
    float CaptureInterval = 0.1f;  // e.g., 0.1 sec ~ 10 FPS

    UPROPERTY(EditAnywhere, Category = "Communication")
    FString DroneID = TEXT("drone1");
};

UCLASS(Blueprintable)
class QUADSIMTOREALITY_API AZMQController : public AActor
{
    GENERATED_BODY()
    
public:
    AZMQController();
    virtual ~AZMQController();

    // Call this to initialize the controller with a target pawn and its drone controller.
    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController, const FZMQConfiguration& Config);

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void SetConfiguration(const FZMQConfiguration& NewConfig);

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    const FZMQConfiguration& GetConfiguration() const { return Configuration; }

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    FVector GetCurrentGoalPosition() const { return CurrentGoalPosition; }

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void SetDroneID(const FString& NewID);

    // This property is used to point to the drone this controller is responsible for.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ZMQ")
    AQuadPawn* TargetPawn;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

private:
    void InitializeImageCapture();
    void ProcessImageCapture();
    void InitializeZMQ();
    void HandleResetCommand();
    void HandleVelocityCommand(zmq::multipart_t& Message);
    void SendStateData();
    void ProcessCommands();
    void CheckAndInitialize();
    TArray<uint8> CompressImageData(const TArray<FColor>& ImageData);

    // ZMQ and image capture members
    zmq::context_t Context;
    TSharedPtr<zmq::socket_t> PublishSocket;
    TSharedPtr<zmq::socket_t> CommandSocket;
    TSharedPtr<zmq::socket_t> ControlSocket;

    UPROPERTY(EditAnywhere, Category = "ZMQ")
    FZMQConfiguration Configuration;

    FTimerHandle ImageCaptureTimerHandle;
    TAtomic<bool> bIsCapturing;
    TAtomic<bool> bIsProcessingCommand;

    // These will be set during Initialize()
    UPROPERTY()
    AQuadPawn* DronePawn;

    UPROPERTY()
    UQuadDroneController* DroneController;

    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    FVector CurrentGoalPosition;

};
