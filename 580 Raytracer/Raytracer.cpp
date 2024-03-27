#include "Raytracer.h"


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
}
