// UIManager.cpp
#include "UI/CoreUI/UIManager.h"
#include "Blueprint/UserWidget.h"
#include "GameFramework/PlayerController.h"

void UUIManager::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    LayerWidgets.Empty();
}

void UUIManager::Deinitialize()
{
    // Clear all layers when shutting down
    for (auto& Pair : LayerWidgets)
    {
        ClearLayer(Pair.Key);
    }
    LayerWidgets.Empty();

    Super::Deinitialize();
}

UUserWidget* UUIManager::PushContentToLayer(APlayerController* OwningPlayer, 
                                          EUILayer Layer,
                                          TSubclassOf<UUserWidget> WidgetClass)
{
    if (!OwningPlayer || !WidgetClass)
        return nullptr;

    // Create the widget
    UUserWidget* NewWidget = CreateWidget<UUserWidget>(OwningPlayer, WidgetClass);
    if (!NewWidget)
        return nullptr;

    // Initialize layer array if it doesn't exist
    if (!LayerWidgets.Contains(Layer))
    {
        LayerWidgets.Add(Layer, TArray<UUserWidget*>());
    }

    // Calculate Z-order based on layer and stack position
    int32 ZOrder = GetBaseZOrderForLayer(Layer) + LayerWidgets[Layer].Num();
    
    // Add to viewport with calculated Z-order
    NewWidget->AddToViewport(ZOrder);
    LayerWidgets[Layer].Add(NewWidget);

    return NewWidget;
}

void UUIManager::PopContentFromLayer(EUILayer Layer)
{
    if (!LayerWidgets.Contains(Layer) || LayerWidgets[Layer].Num() == 0)
        return;

    int32 LastIndex = LayerWidgets[Layer].Num() - 1;
    if (UUserWidget* Widget = LayerWidgets[Layer][LastIndex])
    {
        Widget->RemoveFromParent();
        LayerWidgets[Layer].RemoveAt(LastIndex);
    }
}

void UUIManager::ClearLayer(EUILayer Layer)
{
    if (!LayerWidgets.Contains(Layer))
        return;

    for (UUserWidget* Widget : LayerWidgets[Layer])
    {
        if (Widget)
        {
            Widget->RemoveFromParent();
        }
    }
    LayerWidgets[Layer].Empty();
}

UUserWidget* UUIManager::GetTopWidgetFromLayer(EUILayer Layer) const
{
    if (!LayerWidgets.Contains(Layer) || LayerWidgets[Layer].Num() == 0)
        return nullptr;

    return LayerWidgets[Layer].Last();
}

void UUIManager::RemoveWidget(UUserWidget* Widget)
{
    if (!Widget)
        return;

    // Find and remove the widget from its layer
    for (auto& Pair : LayerWidgets)
    {
        if (Pair.Value.Remove(Widget) > 0)
        {
            Widget->RemoveFromParent();
            break;
        }
    }
}

bool UUIManager::IsLayerEmpty(EUILayer Layer) const
{
    return !LayerWidgets.Contains(Layer) || LayerWidgets[Layer].Num() == 0;
}

int32 UUIManager::GetBaseZOrderForLayer(EUILayer Layer) const
{
    // Each layer gets a range of 1000 Z-order values
    return static_cast<int32>(Layer) * 1000;
}