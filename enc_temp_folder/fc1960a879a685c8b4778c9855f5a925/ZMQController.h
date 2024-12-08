#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ZMQController.generated.h"

// Forward declarations
class AQuadPawn;
class UQuadDroneController;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class AActor;

namespace zmq {
    class context_t;
    class socket_t;
    class message_t;
    class multipart_t;
    enum class socket_type : int;
    enum class recv_flags : int;
    enum class send_flags : int;
    class error_t;
}

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class QUADSIMTOREALITY_API UZMQController : public UActorComponent
{
    GENERATED_BODY()

public:
    UZMQController();

    void Initialize(AQuadPawn* InPawn, UQuadDroneController* InDroneController);

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void SetUpdateInterval(float NewInterval) { UpdateInterval = NewInterval; }

    UFUNCTION(BlueprintCallable, Category = "ZMQ")
    void SetDroneID(const FString& NewID) { DroneID = NewID; }

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

private:
    // Setup functions
    void InitializeZMQ();
    void SetupImageCapture();
    void CleanupZMQ();

    // Communication functions
    void CaptureAndSendImage();
    void SendImageData(const TArray<uint8>& CompressedBitmap);
    void ReceiveVelocityCommand();
    void HandleResetCommand();
    void HandleVelocityCommand(zmq::multipart_t& multipart);
    void SendData();

    // ZMQ members
    static zmq::context_t* SharedZMQContext;
    static zmq::socket_t* SharedZMQSocket;
    static int32 SharedResourceRefCount;

    zmq::socket_t* ZMQSocket;
    zmq::socket_t* CommandSocket;
    zmq::socket_t* ControlSocket;

    // Component references
    UPROPERTY()
    UQuadDroneController* DroneController;

    UPROPERTY()
    AQuadPawn* DronePawn;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureComponent;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    UPROPERTY()
    AActor* SM_PillarFrameActor;

    // Configuration
    UPROPERTY(EditAnywhere, Category = "ZMQ Configuration")
    float UpdateInterval;

    UPROPERTY(EditAnywhere, Category = "ZMQ Configuration")
    FString DroneID;

    // State variables
    float TimeSinceLastUpdate;
    bool bIsActive;
    FVector InitialPosition;
    FVector GoalPosition;

    // Constants
    static constexpr int32 DEFAULT_PUB_PORT = 5555;
    static constexpr int32 DEFAULT_SUB_PORT = 5556;
    static constexpr float DEFAULT_UPDATE_INTERVAL = 0.0001f; // Adjusted to match original value
};


//#pragma once
//
//
//#include "CoreMinimal.h"
//#include "Components/ActorComponent.h"
//#include "Components/SceneCaptureComponent2D.h"
//#include "Engine/TextureRenderTarget2D.h"
//#include "QuadSimToReality/QuadDroneController.h"
//#include "ZMQController.generated.h"
//
//// Forward declarations for ZeroMQ
//namespace zmq {
//    class context_t;
//    class socket_t;
//    class message_t;
//    class multipart_t;
//    enum class socket_type : int;
//    enum class recv_flags : int;
//    enum class send_flags : int;
//    class error_t;
//}
//
//UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
//class QUADSIMTOREALITY_API UZMQController : public UActorComponent {
//    GENERATED_BODY()
//
//public:
//    UZMQController();
//
//
//protected:
//    virtual void BeginPlay() override;
//    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
//    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
//
//private:
//    // Shared ZeroMQ context
//    static zmq::context_t* SharedZMQContext;
//
//    // Shared ZeroMQ PUB socket
//    static zmq::socket_t* SharedZMQSocket;
//
//    zmq::socket_t* ZMQSocket;
//    zmq::socket_t* CommandSocket;
//    zmq::socket_t* ControlSocket;
//
//    void CaptureAndSendImage();
//    void SendImageData(const TArray<uint8>& CompressedBitmap);
//    void ReceiveVelocityCommand();
//    void SendData();
//
//    UQuadDroneController* DroneController;
//
//    UPROPERTY()
//    USceneCaptureComponent2D* SceneCaptureComponent;
//
//    UPROPERTY()
//    AActor* SM_PillarFrameActor;
//
//    UPROPERTY()
//    UTextureRenderTarget2D* RenderTarget;
//
//    float UpdateInterval;
//    float TimeSinceLastUpdate;
//
//    static int32 SharedResourceRefCount;
//
//    // Add an active flag
//    bool bIsActive;
//
//    AQuadPawn* DronePawn; 
//    FVector InitialPosition;
//    FVector GoalPosition; 
//    FString DroneID;
//};