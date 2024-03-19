#pragma once
#include "../Common/module_interfaces/ntshengn_physics_module_interface.h"

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Default Physics Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns an IntersectionInformation structure containing information about the intersection
		IntersectionInformation intersect(const ColliderShape* shape1, const ColliderShape* shape2);

		// Returns a RaycastInformation structure containinng informationn about the raycast
		RaycastInformation raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax, const ColliderShape* shape);
		// Returns a list of RaycastInformation structures containing information about the hit entities
		std::vector<std::pair<Entity, RaycastInformation>> raycastAll(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax);
	};

}