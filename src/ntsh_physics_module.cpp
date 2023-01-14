#include "ntsh_physics_module.h"
#include "../external/Module/utils/ntsh_module_defines.h"
#include "../external/Module/utils/ntsh_dynamic_library.h"
#include "../external/Common/utils/ntsh_engine_defines.h"
#include "../external/Common/utils/ntsh_engine_enums.h"

void NutshellPhysicsModule::init() {
	NTSH_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

void NutshellPhysicsModule::update(double dt) {
	NTSH_UNUSED(dt);
	NTSH_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

void NutshellPhysicsModule::destroy() {
	NTSH_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

bool NutshellPhysicsModule::intersect(Ntsh::ColliderShape* shape1, Ntsh::ColliderShape* shape2) {
	NTSH_UNUSED(shape1);
	NTSH_UNUSED(shape2);
	NTSH_MODULE_FUNCTION_NOT_IMPLEMENTED();
	return false;
}

extern "C" NTSH_MODULE_API NutshellPhysicsModuleInterface* createModule() {
	return new NutshellPhysicsModule;
}

extern "C" NTSH_MODULE_API void destroyModule(NutshellPhysicsModuleInterface* m) {
	delete m;
}