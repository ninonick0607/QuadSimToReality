#include "Core/DroneGlobalState.h"
#include "Pawns/QuadPawn.h"
#include "Controllers/QuadDroneController.h"
#include "imgui.h"
#include "Kismet/GameplayStatics.h"

// Initialize the static pointer to nullptr
UDroneGlobalState* UDroneGlobalState::SingletonInstance = nullptr;

UDroneGlobalState* UDroneGlobalState::Get()
{
    if (!SingletonInstance)
    {
        SingletonInstance = NewObject<UDroneGlobalState>();
        SingletonInstance->AddToRoot();
    }
    return SingletonInstance;
}

// Standard constructor for a UObject
UDroneGlobalState::UDroneGlobalState()
    : DesiredVelocity(FVector::ZeroVector)
    , BoundController(nullptr)
    , DroneCounter(1) 
{
}

void UDroneGlobalState::SetDesiredVelocity(const FVector& NewVelocity)
{
    DesiredVelocity = NewVelocity;

}

void UDroneGlobalState::BindController(UQuadDroneController* Controller)
{
    BoundController = Controller;
}

void UDroneGlobalState::UnbindController()
{
    BoundController = nullptr;
}

void UDroneGlobalState::RegisterPawn(AQuadPawn* InPawn)
{
    if (InPawn)
    {
        AllDrones.Add(InPawn);
    }
}


void UDroneGlobalState::DrawDroneManagerWindow(UWorld* InWorld, const FString& InDroneID)
{
    ImGui::Begin("Drone Manager");
    
    ImGui::Text("Caller DroneID: %s", TCHAR_TO_UTF8(*InDroneID));
    ImGui::Separator();

    ImGui::Text("Select which drone to possess:");

    static TArray<const char*> DroneLabels;
    DroneLabels.Reset();

    // Clean out any invalid references
    for (int i = AllDrones.Num() - 1; i >= 0; i--)
    {
        if (!AllDrones[i].IsValid())
        {
            AllDrones.RemoveAt(i);
        }
    }

    for (int i = 0; i < AllDrones.Num(); i++)
    {
        FString Label = FString::Printf(TEXT("Drone%d##%d"), i + 1, i);
        DroneLabels.Add(TCHAR_TO_UTF8(*Label));
    }

    // If we have any valid drone labels...
    if (DroneLabels.Num() > 0)
    {
        // Ensure index is valid
        if (SelectedDroneIndex >= DroneLabels.Num())
        {
            SelectedDroneIndex = 0;
        }

        // ImGui combo
        if (ImGui::BeginCombo("Active Drone", DroneLabels[SelectedDroneIndex]))
        {
            for (int i = 0; i < DroneLabels.Num(); i++)
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

        // After the combo box, check if the currently possessed pawn is different
        APlayerController* PC = UGameplayStatics::GetPlayerController(InWorld, 0);
        AQuadPawn* CurrentPawn = Cast<AQuadPawn>(PC->GetPawn());
        if (AllDrones.IsValidIndex(SelectedDroneIndex))
        {
            AQuadPawn* SelectedPawn = AllDrones[SelectedDroneIndex].Get();
            // If the user chooses a different drone, possess it
            if (SelectedPawn && SelectedPawn != CurrentPawn)
            {
                PC->Possess(SelectedPawn);
            }
        }
    }
    else
    {
        ImGui::Text("No drones spawned yet.");
    }

    ImGui::End();
}