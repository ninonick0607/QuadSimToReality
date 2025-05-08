// QuadSimCoreModule.cpp
// Module implementation for the QuadSimCore plugin module
#include "Modules/ModuleManager.h"

class FQuadSimCoreModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        // Module startup logic (if any) goes here
    }

    virtual void ShutdownModule() override
    {
        // Module shutdown logic (if any) goes here
    }
};

IMPLEMENT_MODULE(FQuadSimCoreModule, QuadSimCore)