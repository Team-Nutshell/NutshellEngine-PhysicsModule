#include "ntsh_physics_module.h"
#include "../external/Module/ntsh_module_defines.h"
#include "../external/Module/ntsh_dynamic_library.h"
#include "../external/Common/ntsh_engine_enums.h"

void NutshellPhysicsModule::init() {
	NTSH_MODULE_WARNING("init() function not implemented.");
}

void NutshellPhysicsModule::update(double dt) {
	NTSH_UNUSED(dt);
	NTSH_MODULE_WARNING("update() function not implemented.");
}

void NutshellPhysicsModule::destroy() {
	NTSH_MODULE_WARNING("destroy() function not implemented.");
}

extern "C" NTSH_MODULE_API NutshellPhysicsModuleInterface* createModule() {
	return new NutshellPhysicsModule;
}

extern "C" NTSH_MODULE_API void destroyModule(NutshellPhysicsModuleInterface* m) {
	delete m;
}