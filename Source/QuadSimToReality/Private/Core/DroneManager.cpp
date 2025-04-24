#include "Core/DroneManager.h"
// Controller includes
#include "Controllers/QuadDroneController.h"
#include "Pawns/QuadPawn.h"
#include "Kismet/GameplayStatics.h"
    #include "Engine/World.h"
    #include "imgui.h"
    #include "Engine/Engine.h"

    // Initialize static or default values
    // Static accessor for the DroneManager in the world
    ADroneManager* ADroneManager::Get(UWorld* World)
    {
        if (!World) return nullptr;
        TArray<AActor*> Found;
        UGameplayStatics::GetAllActorsOfClass(World, ADroneManager::StaticClass(), Found);
        return Found.Num() > 0 ? Cast<ADroneManager>(Found[0]) : nullptr;
    }

    ADroneManager::ADroneManager()
    {
        PrimaryActorTick.bCanEverTick = true;
        SelectedDroneIndex = 0;
    }

    void ADroneManager::BeginPlay()
    {
       Super::BeginPlay();
       // Ensure QuadPawnClass is set; fallback to C++ base class if unset
       if (!QuadPawnClass)
       {
           QuadPawnClass = AQuadPawn::StaticClass();
           UE_LOG(LogTemp, Warning, TEXT("QuadPawnClass not set; defaulting to C++ AQuadPawn class"));
       }
        
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
        // Initialize last spawn location to the most recently found drone (assumed ground level)
        if (AllDrones.Num() > 0)
        {
            if (AQuadPawn* LastPawn = AllDrones.Last().Get())
            {
                LastSpawnLocation = LastPawn->GetActorLocation();
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

    
    // Register a Quad Drone Controller to receive global flight mode broadcasts
    void ADroneManager::RegisterDroneController(UQuadDroneController* Controller)
    {
        if (Controller)
        {
            OnGlobalFlightModeChanged.AddUObject(Controller, &UQuadDroneController::SetFlightMode);
        }
    }
    
    // Enable or disable swarm mode
    void ADroneManager::SetSwarmMode(bool bEnable)
    {
        bSwarmMode = bEnable;
    }
    bool ADroneManager::IsSwarmMode() const
    {
        return bSwarmMode;
    }
    
    // Get the index of the given drone within AllDrones
    int32 ADroneManager::GetDroneIndex(AQuadPawn* Pawn) const
    {
        for (int32 i = 0; i < AllDrones.Num(); ++i)
        {
            if (AllDrones[i].Get() == Pawn)
            {
                return i;
            }
        }
        return INDEX_NONE;
    }

    void ADroneManager::Tick(float DeltaTime)
    {
        Super::Tick(DeltaTime);

        // Clean up any invalid drone entries.
        for (int32 i = AllDrones.Num() - 1; i >= 0; i--)
        {
            if (!AllDrones[i].IsValid())
            {
                AllDrones.RemoveAt(i);
            }
        }


        // Prepare the drone labels for the ImGui interface.
        ImGui::Begin("Global Drone Manager");
        // Swarm vs. Independent mode toggle
        ImGui::Checkbox("Swarm Mode", &bSwarmMode);
        ImGui::Text("Select which drone to possess:");
        
        int32 NumDrones = AllDrones.Num();
        if (NumDrones > 0)
        {
            // Clamp the selected index to valid range
            if (SelectedDroneIndex < 0 || SelectedDroneIndex >= NumDrones)
            {
                SelectedDroneIndex = 0;
            }
            // Display current drone ID
            AQuadPawn* CurrentPawn = AllDrones[SelectedDroneIndex].Get();
            FString CurrentID = CurrentPawn ? CurrentPawn->DroneID : FString::Printf(TEXT("Drone%d"), SelectedDroneIndex + 1);
            // Dropdown to select active drone
            if (ImGui::BeginCombo("Active Drone", TCHAR_TO_UTF8(*CurrentID)))
            {
                for (int32 i = 0; i < NumDrones; ++i)
                {
                    AQuadPawn* Drone = AllDrones[i].Get();
                    FString DroneID = Drone ? Drone->DroneID : FString::Printf(TEXT("Drone%d"), i + 1);
                    FString Label = FString::Printf(TEXT("%s##%d"), *DroneID, i);
                    bool bSelected = (SelectedDroneIndex == i);
                    if (ImGui::Selectable(TCHAR_TO_UTF8(*Label), bSelected))
                    {
                        SelectedDroneIndex = i;
                    }
                    if (bSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            // Possess selected drone
            if (APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0))
            {
                AQuadPawn* NewPawn = AllDrones[SelectedDroneIndex].Get();
                if (NewPawn && PC->GetPawn() != NewPawn)
                {
                    PC->Possess(NewPawn);
                }
            }
            // Spawn new drone offset from selected
            if (ImGui::Button("Spawn Drone"))
            {
                // Spawn next drone 3 meters (300 cm) away from last spawn, on the same ground level
                FVector SpawnLocation = LastSpawnLocation + FVector(300.f, 0.f, 0.f);
                // Maintain ground (Z) from last spawn
                SpawnLocation.Z = LastSpawnLocation.Z;
                // Use rotation of last spawned drone, if available
                FRotator SpawnRotation = FRotator::ZeroRotator;
                if (AllDrones.Num() > 0)
                {
                    if (AQuadPawn* LastPawn = AllDrones.Last().Get())
                    {
                        SpawnRotation = LastPawn->GetActorRotation();
                    }
                }
                // Spawn the drone and update last spawn location
                if (AQuadPawn* NewDrone = SpawnDrone(SpawnLocation, SpawnRotation))
                {
                    UE_LOG(LogTemp, Display, TEXT("Spawned new drone at %s"), *SpawnLocation.ToString());
                    LastSpawnLocation = SpawnLocation;
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
        if (!QuadPawnClass)
        {
            UE_LOG(LogTemp, Warning, TEXT("QuadPawnClass not set in DroneManager!"));
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
