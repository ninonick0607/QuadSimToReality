#include "SPIDPlotWidget.h"
#include "Rendering/DrawElements.h"

void SPIDPlotWidget::Construct(const FArguments& InArgs)
{
    // Initialization if needed
}

void SPIDPlotWidget::UpdatePIDData(const TArray<float>& InTimeData, const TArray<float>& InPIDData)
{
    // Copy data into the widget's arrays
    TimeData = InTimeData;
    PIDData = InPIDData;

    // Request the widget to redraw
    Invalidate(EInvalidateWidget::Paint);
}

int32 SPIDPlotWidget::OnPaint(
    const FPaintArgs& Args,
    const FGeometry& AllottedGeometry,
    const FSlateRect& MyCullingRect,
    FSlateWindowElementList& OutDrawElements,
    int32 LayerId,
    const FWidgetStyle& InWidgetStyle,
    bool bParentEnabled) const
{
    // Call the parent function to ensure proper behavior
    int32 RetLayerId = SCompoundWidget::OnPaint(
        Args, AllottedGeometry, MyCullingRect, OutDrawElements,
        LayerId, InWidgetStyle, bParentEnabled);

    // Check if we have enough data to draw
    if (TimeData.Num() < 2 || PIDData.Num() < 2)
    {
        return RetLayerId;
    }

    // Prepare points for the graph
    TArray<FVector2D> Points;
    float Width = AllottedGeometry.GetLocalSize().X;
    float Height = AllottedGeometry.GetLocalSize().Y;

    // Find min and max values for normalization
    float MinTime = TimeData[0];
    float MaxTime = TimeData.Last();
    float MinValue = PIDData[0];
    float MaxValue = PIDData[0];
    for (float Value : PIDData)
    {
        if (Value < MinValue) MinValue = Value;
        if (Value > MaxValue) MaxValue = Value;
    }
    float TimeRange = MaxTime - MinTime;
    float ValueRange = MaxValue - MinValue;
    ValueRange = ValueRange > 0 ? ValueRange : 1.0f; // Prevent division by zero

    // Normalize and scale points to widget size
    for (int32 i = 0; i < TimeData.Num(); ++i)
    {
        float NormalizedTime = (TimeData[i] - MinTime) / TimeRange;
        float NormalizedValue = (PIDData[i] - MinValue) / ValueRange;
        float X = NormalizedTime * Width;
        float Y = Height - (NormalizedValue * Height);
        Points.Add(FVector2D(X, Y));
    }

    // Draw the line graph
    FSlateDrawElement::MakeLines(
        OutDrawElements,
        RetLayerId,
        AllottedGeometry.ToPaintGeometry(),
        Points,
        ESlateDrawEffect::None,
        FLinearColor::Green,
        true, 2.0f);

    return RetLayerId + 1;
}
