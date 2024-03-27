#include "Raytracer.h"

/// <summary>
/// For float comparison
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool Raytracer::NearlyEquals(float a, float b) {
	return (std::abs(a - b) < EPSILON);
}

/// <summary>
/// Takes in a ray and computes intersection with scene, if has hit output hit point in world space and normal in hitInfo struct
/// </summary>
/// <param name="ray"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::Raycast(Ray& ray, RaycastHitInfo& hitInfo) {

}


/// <summary>
/// Möller–Trumbore intersection algorithm, given a ray and triangle, test intersection
/// </summary>
/// <param name="ray"></param>
/// <param name="triangle"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::RaycastTriangle(Ray& ray, Triangle& triangle, RaycastHitInfo hitInfo) {
	//Image a plane where the triangle lies on, the ray will intersect with the plane if the plane is in front of the ray
	//Once intersection happens with the plane, we can then figure out if the intersection point is inside the triangle
	//Even if a plane is in front of the ray, the ray will miss it if it's parallel to the ray

	//Get plane normal, which will be the same for the triangle normal since triangle lies flat on the imaginary plane
	Vector3 edge1 = triangle.v1.vertexPos - triangle.v0.vertexPos;
	Vector3 edge2 = triangle.v2.vertexPos - triangle.v0.vertexPos;
	Vector3 planeNormal = Vector3::cross(edge1, edge2);
	planeNormal.normalize();

	float planeNormalDotRayDir = Vector3::dot(planeNormal, ray.direction);

	//Dot product of two vectors is 0 means they are perpendicular to each other, 0, and 1 are pointing along same dir or opposite
	//Perpendicular with the normal means no intersection and you have to go along the normal to point to the plane
	if (NearlyEquals(planeNormalDotRayDir, 0)) {
		return false;
	}
}
