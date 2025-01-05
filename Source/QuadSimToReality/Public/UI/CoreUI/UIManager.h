// UIManager.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "UIManager.generated.h"

// Define UI layers - lower values are rendered first
UENUM(BlueprintType)
enum class EUILayer : uint8
{
	None        UMETA(DisplayName = "None"),
	Background  UMETA(DisplayName = "Background"),
	Game        UMETA(DisplayName = "Game"),
	Menu        UMETA(DisplayName = "Menu"),
	Overlay     UMETA(DisplayName = "Overlay"),
	Modal       UMETA(DisplayName = "Modal"),
	Debug       UMETA(DisplayName = "Debug")
};

UCLASS()
class QUADSIMTOREALITY_API UUIManager : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	// Initialize and clean up
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	// Main widget management functions
	UFUNCTION(BlueprintCallable, Category = "UI")
	UUserWidget* PushContentToLayer(APlayerController* OwningPlayer, 
								  EUILayer Layer,
								  TSubclassOf<UUserWidget> WidgetClass);

	UFUNCTION(BlueprintCallable, Category = "UI")
	void PopContentFromLayer(EUILayer Layer);

	UFUNCTION(BlueprintCallable, Category = "UI")
	void ClearLayer(EUILayer Layer);

	UFUNCTION(BlueprintCallable, Category = "UI")
	UUserWidget* GetTopWidgetFromLayer(EUILayer Layer) const;

	// Helper functions
	UFUNCTION(BlueprintCallable, Category = "UI")
	void RemoveWidget(UUserWidget* Widget);

	UFUNCTION(BlueprintPure, Category = "UI")
	bool IsLayerEmpty(EUILayer Layer) const;
	

private:
	// Store widgets for each layer
	TMap<EUILayer, TArray<UUserWidget*>> LayerWidgets;

	// Helper function to get Z-Order for a layer
	int32 GetBaseZOrderForLayer(EUILayer Layer) const;
};
