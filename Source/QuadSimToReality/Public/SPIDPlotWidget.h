#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

/**
 * A custom Slate widget for displaying PID plots.
 */
class QUADSIMTOREALITY_API SPIDPlotWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPIDPlotWidget)
	{}
	SLATE_END_ARGS()

	/** Constructs this widget with InArgs */
	void Construct(const FArguments& InArgs);

	/** Overrides the OnPaint function to draw the plot */
	virtual int32 OnPaint(
		const FPaintArgs& Args,
		const FGeometry& AllottedGeometry,
		const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements,
		int32 LayerId,
		const FWidgetStyle& InWidgetStyle,
		bool bParentEnabled) const override;

	/** Updates the PID data */
	void UpdatePIDData(const TArray<float>& InTimeData, const TArray<float>& InPIDData);

private:
	/** Arrays to hold the time and PID data */
	TArray<float> TimeData;
	TArray<float> PIDData;
};
