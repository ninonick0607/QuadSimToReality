// QuadSimToReality.h - Primary header for the QuadSimToReality game module
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * Main module class for the QuadSimToReality game module.
 */
class FQuadSimToRealityModule : public IModuleInterface
{
public:
    /** Called right after the module DLL has been loaded and the module has been initialized */
    virtual void StartupModule() override;

    /** Called before the module is unloaded, right before the module DLL is unloaded */
    virtual void ShutdownModule() override;
};