#include "ntshengn_physics_module.h"
#include "../Module/utils/ntshengn_module_defines.h"
#include "../Module/utils/ntshengn_dynamic_library.h"
#include "../Common/utils/ntshengn_defines.h"
#include "../Common/utils/ntshengn_enums.h"

void NtshEngn::PhysicsModule::init() {
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

void NtshEngn::PhysicsModule::update(double dt) {
	NTSHENGN_UNUSED(dt);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

void NtshEngn::PhysicsModule::destroy() {
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderShape* shape1, const ColliderShape* shape2) {
	NTSHENGN_UNUSED(shape1);
	NTSHENGN_UNUSED(shape2);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	IntersectionInformation intersectionInformation;
	intersectionInformation.hasIntersected = false;

	return intersectionInformation;
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}