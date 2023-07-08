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
		NtshEngn::IntersectionInformation intersect(const ColliderShape* shape1, const ColliderShape* shape2);
	};

}