// QuadSimToReality.h
#pragma once
#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FQuadSimToRealityModule : public FDefaultGameModuleImpl   // ← changed
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
