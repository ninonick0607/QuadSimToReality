    #include "Core/DroneManager.h"
    #include "Pawns/QuadPawn.h"
    #include "Controllers/ZMQController.h"
    #include "Kismet/GameplayStatics.h"
    #include "Engine/World.h"
    #include "imgui.h"
    #include "Engine/Engine.h"

    ADroneManager::ADroneManager()
    {
        PrimaryActorTick.bCanEverTick = true;
        SelectedDroneIndex = 0;
    }

    void ADroneManager::BeginPlay()
    {
        Super::BeginPlay();
        
        if (UWorld* World = GetWorld())
        {
            // Subscribe to actor-spawn events.
            OnActorSpawnedHandle = World->AddOnActorSpawnedHandler(
                FOnActorSpawned::FDelegate::CreateLambda([this](AActor* SpawnedActor)
                {
                    this->OnActorSpawned(SpawnedActor);
                })
            );
        }

        // Populate the list with any already existing QuadPawns.
        TArray<AActor*> FoundDrones;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AQuadPawn::StaticClass(), FoundDrones);
        for (AActor* Actor : FoundDrones)
        {
            if (AQuadPawn* Pawn = Cast<AQuadPawn>(Actor))
            {
                AllDrones.Add(Pawn);
            }
        }
    }

    void ADroneManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
    {
        if (UWorld* World = GetWorld())
        {
            World->RemoveOnActorSpawnedHandler(OnActorSpawnedHandle);
        }
        Super::EndPlay(EndPlayReason);
    }

    void ADroneManager::OnActorSpawned(AActor* SpawnedActor)
    {
        if (AQuadPawn* Pawn = Cast<AQuadPawn>(SpawnedActor))
        {
            AllDrones.Add(Pawn);
        }
    }

    void ADroneManager::RegisterZMQController(AZMQController* Controller)
    {
        if (Controller)
        {
            AllZMQControllers.Add(Controller);
            UE_LOG(LogTemp, Display, TEXT("DroneManager: Registered ZMQController with DroneID: %s"), 
                   *Controller->GetConfiguration().DroneID);
        }
    }

    void ADroneManager::Tick(float DeltaTime)
    {
        Super::Tick(DeltaTime);

        // Clean up any invalid entries.
        for (int32 i = AllDrones.Num() - 1; i >= 0; i--)
        {
            if (!AllDrones[i].IsValid())
            {
                AllDrones.RemoveAt(i);
            }
        }

        ImGui::Begin("Global Drone Manager");
        ImGui::Text("Select which drone to possess:");

        TArray<const char*> DroneLabels;
        DroneLabels.Reset();

        for (int32 i = 0; i < AllDrones.Num(); i++)
        {
            AQuadPawn* Drone = AllDrones[i].Get();
            FString DroneID;
            if (Drone)
            {
                // Look for the dedicated ZMQController for this drone.
                AZMQController* ZMQControllerForDrone = nullptr;
                TArray<AActor*> FoundControllers;
                UGameplayStatics::GetAllActorsOfClass(GetWorld(), AZMQController::StaticClass(), FoundControllers);
                for (AActor* Actor : FoundControllers)
                {
                    AZMQController* ZMQ = Cast<AZMQController>(Actor);
                    if (ZMQ && ZMQ->TargetPawn == Drone)
                    {
                        ZMQControllerForDrone = ZMQ;
                        break;
                    }
                }
                if (ZMQControllerForDrone)
                {
                    DroneID = ZMQControllerForDrone->GetConfiguration().DroneID;
                }
                else
                {
                    DroneID = FString::Printf(TEXT("Drone%d"), i + 1);
                }
            }
            else
            {
                DroneID = FString::Printf(TEXT("Drone%d"), i + 1);
            }
            FString Label = FString::Printf(TEXT("%s##%d"), *DroneID, i);
            DroneLabels.Add(TCHAR_TO_UTF8(*Label));
        }

        if (DroneLabels.Num() > 0)
        {
            if (SelectedDroneIndex >= DroneLabels.Num())
            {
                SelectedDroneIndex = 0;
            }

            if (ImGui::BeginCombo("Active Drone", DroneLabels[SelectedDroneIndex]))
            {
                for (int32 i = 0; i < DroneLabels.Num(); i++)
                {
                    ImGui::PushID(i);
                    bool bSelected = (SelectedDroneIndex == i);
                    if (ImGui::Selectable(DroneLabels[i], bSelected))
                    {
                        SelectedDroneIndex = i;
                    }
                    if (bSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }

            // Possess the selected drone.
            APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
            if (PC && AllDrones.IsValidIndex(SelectedDroneIndex))
            {
                AQuadPawn* SelectedPawn = AllDrones[SelectedDroneIndex].Get();
                if (SelectedPawn && PC->GetPawn() != SelectedPawn)
                {
                    PC->Possess(SelectedPawn);
                }
            }

            if (ImGui::Button("Spawn Drone"))
            {
                if (AllDrones.IsValidIndex(SelectedDroneIndex))
                {
                    AQuadPawn* SelectedPawn = AllDrones[SelectedDroneIndex].Get();
                    if (SelectedPawn)
                    {
                        const float SpawnOffsetDistance = 200.f;
                        FVector RightOffset = SelectedPawn->GetActorRightVector() * SpawnOffsetDistance;
                        FVector SpawnLocation = SelectedPawn->GetActorLocation() + RightOffset;
                        FRotator SpawnRotation = SelectedPawn->GetActorRotation();

                        // Spawn a new drone (and its corresponding ZMQController).
                        AQuadPawn* NewDrone = SpawnDrone(SpawnLocation, SpawnRotation);
                        if (NewDrone)
                        {
                            UE_LOG(LogTemp, Display, TEXT("Spawned new drone at %s"), *SpawnLocation.ToString());
                        }
                    }
                }
            }
        }
        else
        {
            ImGui::Text("No drones spawned yet.");
        }

        ImGui::End();
    }

    AQuadPawn* ADroneManager::SpawnDrone(const FVector& SpawnLocation, const FRotator& SpawnRotation)
    {
        if (!QuadPawnClass || !ZMQControllerClass)
        {
            UE_LOG(LogTemp, Warning, TEXT("QuadPawnClass or ZMQControllerClass not set in DroneManager!"));
            return nullptr;
        }

        UWorld* World = GetWorld();
        if (World)
        {
            FActorSpawnParameters SpawnParams;
            SpawnParams.Owner = this;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            
            // Spawn the drone.
            AQuadPawn* NewDrone = World->SpawnActor<AQuadPawn>(QuadPawnClass, SpawnLocation, SpawnRotation, SpawnParams);
            if (NewDrone)
            {
                // Calculate a unique offset based o    n the current number of drones.
                int32 DroneIndex = AllDrones.Num();
                FZMQConfiguration Config;
                Config.PublishPort = 5557 + DroneIndex;
                Config.CommandPort = 5556 + DroneIndex;
                Config.ControlPort = 5558 + DroneIndex;
                Config.DroneID = NewDrone->GetName();
                
                // Spawn the dedicated ZMQController for this drone after a short delay.
                FTimerHandle TimerHandle;
                World->GetTimerManager().SetTimer(TimerHandle, [this, NewDrone, SpawnLocation, SpawnRotation, SpawnParams, Config]()
                {
                    AZMQController* NewZMQController = GetWorld()->SpawnActor<AZMQController>(ZMQControllerClass, SpawnLocation, SpawnRotation, SpawnParams);
                    if (NewZMQController)
                    {
                        NewZMQController->Initialize(NewDrone, NewDrone->QuadController, Config);
                    }
                }, 0.2f, false);
            }
            return NewDrone;
        }
        return nullptr;
    }



    TArray<AQuadPawn*> ADroneManager::GetDroneList() const
    {
        TArray<AQuadPawn*> DroneList;
        for (const TWeakObjectPtr<AQuadPawn>& DronePtr : AllDrones)
        {
            if (AQuadPawn* Drone = DronePtr.Get())
            {
                DroneList.Add(Drone);
            }
        }
        return DroneList;
    }
