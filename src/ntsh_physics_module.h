#pragma once
#include "../external/Common/module_interfaces/ntsh_physics_module_interface.h"

class NutshellPhysicsModule : public NutshellPhysicsModuleInterface {
public:
    NutshellPhysicsModule() : NutshellPhysicsModuleInterface("Nutshell Physics Test Module") {}

    void init();
    void update(double dt);
    void destroy();
};