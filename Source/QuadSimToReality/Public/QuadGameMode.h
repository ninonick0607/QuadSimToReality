// QuadGameMode.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "QuadGameMode.generated.h"

UCLASS()
class QUADSIMTOREALITY_API AQuadGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AQuadGameMode();

protected:
    virtual void BeginPlay() override;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "UI")
    TSubclassOf<class UQuadUI> MainMenuWidgetClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "UI")
    TSubclassOf<class UFlightModeHUD> FlightModeHUDClass;
};
