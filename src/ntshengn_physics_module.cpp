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

	return IntersectionInformation();
}

NtshEngn::RaycastInformation NtshEngn::PhysicsModule::raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax, const ColliderShape* shape) {
	NTSHENGN_UNUSED(rayOrigin);
	NTSHENGN_UNUSED(rayDirection);
	NTSHENGN_UNUSED(tMin);
	NTSHENGN_UNUSED(tMax);
	NTSHENGN_UNUSED(shape);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return RaycastInformation();
}

std::vector<std::pair<NtshEngn::Entity, NtshEngn::RaycastInformation>> NtshEngn::PhysicsModule::raycastAll(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax) {
	NTSHENGN_UNUSED(rayOrigin);
	NTSHENGN_UNUSED(rayDirection);
	NTSHENGN_UNUSED(tMin);
	NTSHENGN_UNUSED(tMax);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return std::vector<std::pair<Entity, RaycastInformation>>();
}

void NtshEngn::PhysicsModule::setConstantForces(const Math::vec3& constantForces) {
	NTSHENGN_UNUSED(constantForces);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getConstantForces() {
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}