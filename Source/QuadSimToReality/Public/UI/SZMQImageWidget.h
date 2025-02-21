#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"

// Forward declarations
class UTextureRenderTarget2D;
struct FSlateDynamicImageBrush;

class SZMQImageWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SZMQImageWidget)
		: _RenderTarget(nullptr)
	{}
	SLATE_ARGUMENT(UTextureRenderTarget2D*, RenderTarget)
SLATE_END_ARGS()

/** Constructs the widget */
void Construct(const FArguments& InArgs);

	/** Updates the render target to display */
	void SetRenderTarget(UTextureRenderTarget2D* InRenderTarget);

	// Override Tick to update FPS and refresh the brush
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;

private:
	/** Returns the dynamic image brush used by the SImage widget */
	const FSlateBrush* GetImageBrush() const;

	/** Returns the FPS display text */
	FText GetFPSDisplayText() const;

	/** The render target to display */
	UTextureRenderTarget2D* RenderTarget;

	/** The dynamic image brush that wraps our render target */
	TSharedPtr<FSlateDynamicImageBrush> ImageBrush;

	// Variables to calculate FPS
	float DeltaTimeAccumulator;
	int32 FrameCount;
	float FPS;
};
