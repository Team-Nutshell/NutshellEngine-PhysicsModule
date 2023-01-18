#pragma once
#include "../external/Common/module_interfaces/ntshengn_physics_module_interface.h"

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Physics Test Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns true if the two shapes are intersecting with each other, else, returns false
		bool intersect(const NtshEngn::ColliderShape& shape1, const NtshEngn::ColliderShape& shape2);
	};

}