#pragma once


#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "QuadSimToReality/QuadDroneController.h"
#include "ZMQController.generated.h"

// Forward declarations for ZeroMQ
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
class QUADSIMTOREALITY_API UZMQController : public UActorComponent {
    GENERATED_BODY()

public:
    UZMQController();


protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // Shared ZeroMQ context
    static zmq::context_t* SharedZMQContext;

    // Shared ZeroMQ PUB socket
    static zmq::socket_t* SharedZMQSocket;

    zmq::socket_t* ZMQSocket;
    zmq::socket_t* CommandSocket;

    void CaptureAndSendImage();
    void SendImageData(const TArray<uint8>& CompressedBitmap);
    void ReceiveVelocityCommand();

    UQuadDroneController* DroneController;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureComponent;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTarget;

    // Declare UpdateInterval before TimeSinceLastUpdate
    float UpdateInterval;
    float TimeSinceLastUpdate;

    // Add reference counting for shared ZMQ context and socket
    static int32 SharedResourceRefCount;

    // Add an active flag
    bool bIsActive;

    FString DroneID;
};