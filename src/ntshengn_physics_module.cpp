#include "ntshengn_physics_module.h"
#include "../external/Module/utils/ntshengn_module_defines.h"
#include "../external/Module/utils/ntshengn_dynamic_library.h"
#include "../external/Common/utils/ntshengn_defines.h"
#include "../external/Common/utils/ntshengn_enums.h"
#include <algorithm>
#include <cmath>

void NtshEngn::PhysicsModule::init() {
}

void NtshEngn::PhysicsModule::update(double dt) {
	NTSHENGN_UNUSED(dt);
}

void NtshEngn::PhysicsModule::destroy() {
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2) {
	if (shape1->getType() == NtshEngn::ColliderShapeType::Sphere) {
		if (shape2->getType() == NtshEngn::ColliderShapeType::Sphere) {
			return intersect(static_cast<const NtshEngn::ColliderSphere*>(shape1), static_cast<const NtshEngn::ColliderSphere*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::AABB) {
			return intersect(static_cast<const NtshEngn::ColliderSphere*>(shape1), static_cast<const NtshEngn::ColliderAABB*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::Capsule) {
			return intersect(static_cast<const NtshEngn::ColliderSphere*>(shape1), static_cast<const NtshEngn::ColliderCapsule*>(shape2));
		}
	}
	else if (shape1->getType() == NtshEngn::ColliderShapeType::AABB) {
		if (shape2->getType() == NtshEngn::ColliderShapeType::Sphere) {
			return intersect(static_cast<const NtshEngn::ColliderAABB*>(shape1), static_cast<const NtshEngn::ColliderSphere*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::AABB) {
			return intersect(static_cast<const NtshEngn::ColliderAABB*>(shape1), static_cast<const NtshEngn::ColliderAABB*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::Capsule) {
			return intersect(static_cast<const NtshEngn::ColliderAABB*>(shape1), static_cast<const NtshEngn::ColliderCapsule*>(shape2));
		}
	}
	else if (shape1->getType() == NtshEngn::ColliderShapeType::Capsule) {
		if (shape2->getType() == NtshEngn::ColliderShapeType::Sphere) {
			return intersect(static_cast<const NtshEngn::ColliderCapsule*>(shape1), static_cast<const NtshEngn::ColliderSphere*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::AABB) {
			return intersect(static_cast<const NtshEngn::ColliderCapsule*>(shape1), static_cast<const NtshEngn::ColliderAABB*>(shape2));
		}
		else if (shape2->getType() == NtshEngn::ColliderShapeType::Capsule) {
			return intersect(static_cast<const NtshEngn::ColliderCapsule*>(shape1), static_cast<const NtshEngn::ColliderCapsule*>(shape2));
		}
	}

	return false;
}

const NtshEngn::ComponentMask NtshEngn::PhysicsModule::getComponentMask() const {
	ComponentMask componentMask;
	componentMask.set(m_ecs->getComponentId<AABBCollidable>());
	componentMask.set(m_ecs->getComponentId<SphereCollidable>());
	componentMask.set(m_ecs->getComponentId<CapsuleCollidable>());

	return componentMask;
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderSphere* sphere1, const NtshEngn::ColliderSphere* sphere2) {
	const nml::vec3 sphere1Center = nml::vec3(sphere1->center[0], sphere1->center[1], sphere1->center[2]);
	const nml::vec3 sphere2Center = nml::vec3(sphere2->center[0], sphere2->center[1], sphere2->center[2]);
	const nml::vec3 centerDiff = sphere1Center - sphere2Center;

	return centerDiff.length() < (sphere1->radius + sphere2->radius);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderAABB* aabb) {
	const float x = std::max(aabb->min[0], std::min(sphere->center[0], aabb->max[0]));
	const float y = std::max(aabb->min[1], std::min(sphere->center[1], aabb->max[1]));
	const float z = std::max(aabb->min[2], std::min(sphere->center[2], aabb->max[2]));

	const float distance = std::sqrtf((x - sphere->center[0]) * (x - sphere->center[0]) +
		(y - sphere->center[1]) * (y - sphere->center[1]) +
		(z - sphere->center[2]) * (z - sphere->center[2]));

	return distance < sphere->radius;
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderCapsule* capsule) {
	const nml::vec3 sphereCenter = nml::vec3(sphere->center[0], sphere->center[1], sphere->center[2]);
	const nml::vec3 capsuleBase = nml::vec3(capsule->base[0], capsule->base[1], capsule->base[2]);
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip[0], capsule->tip[1], capsule->tip[2]);

	const float squareDistance = squareDistancePointSegment(sphereCenter, capsuleBase, capsuleTip);

	const float radius = sphere->radius + capsule->radius;

	return squareDistance <= (radius * radius);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderAABB* aabb1, const NtshEngn::ColliderAABB* aabb2) {
	return aabb1->min[0] <= aabb2->max[0] &&
		aabb1->max[0] >= aabb2->min[0] &&
		aabb1->min[1] <= aabb2->max[1] &&
		aabb1->max[1] >= aabb2->min[1] &&
		aabb1->min[2] <= aabb2->max[2] &&
		aabb1->max[2] >= aabb2->min[2];
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderCapsule* capsule) {
	const nml::vec3 aabbMin = nml::vec3(aabb->min[0], aabb->min[1], aabb->min[2]);
	const nml::vec3 aabbMax = nml::vec3(aabb->max[0], aabb->max[1], aabb->max[2]);

	const nml::vec3 mmm = nml::vec3(aabbMin.x, aabbMin.y, aabbMin.z);
	const nml::vec3 Mmm = nml::vec3(aabbMax.x, aabbMin.y, aabbMin.z);
	const nml::vec3 mMm = nml::vec3(aabbMin.x, aabbMax.y, aabbMin.z);
	const nml::vec3 MMm = nml::vec3(aabbMax.x, aabbMax.y, aabbMin.z);
	const nml::vec3 mmM = nml::vec3(aabbMin.x, aabbMin.y, aabbMax.z);
	const nml::vec3 MmM = nml::vec3(aabbMax.x, aabbMin.y, aabbMax.z);
	const nml::vec3 mMM = nml::vec3(aabbMin.x, aabbMax.y, aabbMax.z);
	const nml::vec3 MMM = nml::vec3(aabbMax.x, aabbMax.y, aabbMax.z);

	const nml::vec3 capsuleBase = nml::vec3(capsule->base[0], capsule->base[1], capsule->base[2]);
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip[0], capsule->tip[1], capsule->tip[2]);

	const std::array<float, 8> pointsSquareDistance = {
		squareDistancePointSegment(mmm, capsuleBase, capsuleTip),
		squareDistancePointSegment(Mmm, capsuleBase, capsuleTip),
		squareDistancePointSegment(mMm, capsuleBase, capsuleTip),
		squareDistancePointSegment(MMm, capsuleBase, capsuleTip),
		squareDistancePointSegment(mmM, capsuleBase, capsuleTip),
		squareDistancePointSegment(MmM, capsuleBase, capsuleTip),
		squareDistancePointSegment(mMM, capsuleBase, capsuleTip),
		squareDistancePointSegment(MMM, capsuleBase, capsuleTip)
	};
	
	const float closestPointSquareDistance = *std::min_element(pointsSquareDistance.begin(), pointsSquareDistance.end());

	return closestPointSquareDistance <= (capsule->radius * capsule->radius);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderCapsule* capsule1, const NtshEngn::ColliderCapsule* capsule2) {
	const nml::vec3 capsule1Base = nml::vec3(capsule1->base[0], capsule1->base[1], capsule1->base[2]);
	const nml::vec3 capsule1Tip = nml::vec3(capsule1->tip[0], capsule1->tip[1], capsule1->tip[2]);
	const nml::vec3 capsule2Base = nml::vec3(capsule2->base[0], capsule2->base[1], capsule2->base[2]);
	const nml::vec3 capsule2Tip = nml::vec3(capsule2->tip[0], capsule2->tip[1], capsule2->tip[2]);

	const nml::vec3 capsule1Normal = nml::normalize(capsule1Tip - capsule1Base);
	const nml::vec3 capsule1LineEndOffset = capsule1Normal * capsule1->radius;
	const nml::vec3 capsule1A = capsule1Base + capsule1LineEndOffset;
	const nml::vec3 capsule1B = capsule1Tip - capsule1LineEndOffset;

	const nml::vec3 capsule2Normal = nml::normalize(capsule2Tip - capsule2Base);
	const nml::vec3 capsule2LineEndOffset = capsule2Normal * capsule2->radius;
	const nml::vec3 capsule2A = capsule2Base + capsule2LineEndOffset;
	const nml::vec3 capsule2B = capsule2Tip - capsule2LineEndOffset;

	const nml::vec3 v0 = capsule2A - capsule1A;
	const nml::vec3 v1 = capsule2B - capsule1A;
	const nml::vec3 v2 = capsule2A - capsule1B;
	const nml::vec3 v3 = capsule2B - capsule1B;

	const float d0 = nml::dot(v0, v0);
	const float d1 = nml::dot(v1, v1);
	const float d2 = nml::dot(v2, v2);
	const float d3 = nml::dot(v3, v3);

	nml::vec3 capsule1Best;
	if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1) {
		capsule1Best = capsule1B;
	}
	else {
		capsule1Best = capsule1A;
	}

	const nml::vec3 capsule2Best = closestPointOnSegment(capsule1Best, capsule2A, capsule2B);
	capsule1Best = closestPointOnSegment(capsule2Best, capsule1A, capsule1B);

	const nml::vec3 diff = capsule1Best - capsule2Best;


	return diff.length() < (capsule1->radius + capsule2->radius);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderSphere* sphere) {
	return intersect(sphere, aabb);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderSphere* sphere) {
	return intersect(sphere, capsule);
}

bool NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderAABB* aabb) {
	return intersect(aabb, capsule);
}

float NtshEngn::PhysicsModule::squareDistancePointSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB) {
	const nml::vec3 ab = segmentB - segmentA;
	const nml::vec3 ap = point - segmentA;
	const nml::vec3 bp = point - segmentB;

	float e = nml::dot(ap, ab);
	if (e <= 0.0f) {
		return nml::dot(ap, ap);
	}

	float f = nml::dot(ab, ab);
	if (e >= f) {
		return nml::dot(bp, bp);
	}

	return nml::dot(ap, ap) - ((e * e) / f);
}

nml::vec3 NtshEngn::PhysicsModule::closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB) {
	const nml::vec3 ab = segmentB - segmentA;
	const nml::vec3 ap = point - segmentA;
	const nml::vec3 bp = point - segmentB;

	float e = dot(ap, ab);
	return segmentA + (std::min(std::max(e, 0.0f), 1.0f) * ab);
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}