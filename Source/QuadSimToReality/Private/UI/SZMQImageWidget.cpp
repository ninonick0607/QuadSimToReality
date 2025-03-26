#include "UI/SZMQImageWidget.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SOverlay.h"
#include "Brushes/SlateDynamicImageBrush.h"

void SZMQImageWidget::Construct(const FArguments& InArgs)
{
	RenderTarget = InArgs._RenderTarget;
	DeltaTimeAccumulator = 0.0f;
	FrameCount = 0;
	FPS = 0.0f;

	// If we have a valid render target, create our dynamic brush.
	if (RenderTarget)
	{
		FString BrushName = FString::Printf(TEXT("SZMQImageBrush_%s"), *RenderTarget->GetName());
		ImageBrush = MakeShareable(new FSlateDynamicImageBrush(
			FName(*BrushName),
			FVector2D(RenderTarget->SizeX, RenderTarget->SizeY)
		));
		ImageBrush->SetResourceObject(RenderTarget);
	}

	ChildSlot
	[
		SNew(SOverlay)
		+ SOverlay::Slot()
		[
			// The main image displaying our render target
			SNew(SImage)
			.Image(this, &SZMQImageWidget::GetImageBrush)
		]
		+ SOverlay::Slot()
		.HAlign(HAlign_Left)
		.VAlign(VAlign_Top)
		[
			// An overlay text block showing FPS
			SNew(STextBlock)
			.Text(this, &SZMQImageWidget::GetFPSDisplayText)
			.ColorAndOpacity(FSlateColor(FLinearColor::Yellow))
			.Margin(FMargin(10.0f))
		]
	];
}

const FSlateBrush* SZMQImageWidget::GetImageBrush() const
{
	return ImageBrush.IsValid() ? ImageBrush.Get() : nullptr;
}

FText SZMQImageWidget::GetFPSDisplayText() const
{
	return FText::FromString(FString::Printf(TEXT("FPS: %.2f"), FPS));
}

void SZMQImageWidget::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);

	// Update FPS calculation
	DeltaTimeAccumulator += InDeltaTime;
	FrameCount++;

	if (DeltaTimeAccumulator >= 1.0f)
	{
		FPS = FrameCount / DeltaTimeAccumulator;
		DeltaTimeAccumulator = 0.0f;
		FrameCount = 0;
	}

	// (Optional) Refresh the brush resource if needed so that the latest render target image is shown.
	if (RenderTarget && ImageBrush.IsValid())
	{
		ImageBrush->SetResourceObject(RenderTarget);
	}
}

void SZMQImageWidget::SetRenderTarget(UTextureRenderTarget2D* InRenderTarget)
{
	RenderTarget = InRenderTarget;
	if (RenderTarget)
	{
		FString BrushName = FString::Printf(TEXT("SZMQImageBrush_%s"), *RenderTarget->GetName());
		if (!ImageBrush.IsValid())
		{
			ImageBrush = MakeShareable(new FSlateDynamicImageBrush(
				FName(*BrushName),
				FVector2D(RenderTarget->SizeX, RenderTarget->SizeY)
			));
		}
		ImageBrush->SetResourceObject(RenderTarget);
	}
}
