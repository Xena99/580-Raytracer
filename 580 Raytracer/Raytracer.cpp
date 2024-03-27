#include "Raytracer.h"
#include <iostream>
/// <summary>
/// For float comparison
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool Raytracer::NearlyEquals(float a, float b) {
	return (std::abs(a - b) < EPSILON);
}

//TODO: For each shape compute model matrix
/// <summary>
/// Takes in a ray and computes intersection with scene, if has hit outputs closest hit with hit point in world space, distance and normal in hitInfo struct
/// </summary>
/// <param name="ray"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::Raycast(Ray& ray, RaycastHitInfo& hitInfo) {
	RaycastHitInfo closestHit;
	bool hasFoundHit = false;
	for (Shape& m : mScene->shapes) {
		Mesh* mesh = mScene->meshMap[m.geometryId];

		//Todo: Compute model matrix from the current shape's transformation
		Matrix modelMatrix;

		for (Triangle& tri : mesh->triangles) {
			RaycastHitInfo tempInfo;
			if (RaycastTriangle(ray, tri, tempInfo, modelMatrix)) {
				if (!hasFoundHit) {
					hasFoundHit = true;
					closestHit = tempInfo;
				}
				else {
					if (tempInfo.distance < closestHit.distance) {
						closestHit = tempInfo;
					}
				}
			}
		}
	}

	if (!hasFoundHit) return false;
	hitInfo = closestHit;
	return true;
}


/// <summary>
/// Möller–Trumbore intersection algorithm, given a ray and triangle, test intersection
/// If successful intersection, outputs world space hit point and hit normal
/// </summary>
/// <param name="ray"></param>
/// <param name="triangle"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::RaycastTriangle(Ray& ray, Triangle& triangle, RaycastHitInfo& hitInfo, Matrix& modelMatrix) {
	//Image a plane where the triangle lies on, the ray will intersect with the plane if the plane is in front of the ray
	//Once intersection happens with the plane, we can then figure out if the intersection point is inside the triangle
	//Even if a plane is in front of the ray, the ray will miss it if it's parallel to the ray

	//Get plane normal, which will be the same for the triangle normal since triangle lies flat on the imaginary plane
	Vector3 _edge1 = triangle.v1.vertexPos - triangle.v0.vertexPos;
	Vector3 _edge2 = triangle.v2.vertexPos - triangle.v0.vertexPos;
	Vector3 planeNormal = Vector3::cross(_edge1, _edge2);
	planeNormal.normalize();

	float planeNormalDotRayDir = Vector3::dot(planeNormal, ray.direction);

	//Dot product of two vectors is 0 means they are perpendicular to each other, 0, and 1 are pointing along same dir or opposite
	//Perpendicular with the normal means no intersection and you have to go along the normal to point to the plane
	if (NearlyEquals(planeNormalDotRayDir, 0)) {
		return false;
	}

	//Ax + By + Cz + D = 0 for plane, where A B C is the normal to plane N = (A, B, C), x y z is any point on the plane
	//We can use any vertex x y z to substitute 
	float D = -(planeNormal.x * triangle.v0.vertexPos.x + planeNormal.y * triangle.v0.vertexPos.y + planeNormal.z * triangle.v0.vertexPos.z);

	//Since P is a point on the plane, we can substitute it into Ax + By + Cz + D = 0, and we already know D from above
	//We can calculate t, which is the distance from ray origin to intersection point
	float t = -(Vector3::dot(planeNormal, ray.origin) + D) / Vector3::dot(planeNormal, ray.direction);

	if (t < 0) {
		//The plane is behind the ray, thus no intersection
		return false;
	}
	//P = RayOrigin + t * RayDirection -> point on plane
	Vector3 pointOnPlane = ray.origin + t * ray.direction;

	//For all three edges, if dot(vector from first vertex of the edge to p, edge) > 0, then it means that the point is left of the edge
	//When point is left of all three edges, the point is inside the triangle

	//However, the orientation of the triangle could mess up the calculation is specified wrong
	//For practical purpose, we can add a step to compute cross product of edge with the point vector
	//And use the directional information to know if it's left or right
	Vector3 edge0 = triangle.v1.vertexPos - triangle.v0.vertexPos;
	Vector3 edge1 = triangle.v2.vertexPos - triangle.v1.vertexPos;
	Vector3 edge2 = triangle.v0.vertexPos - triangle.v2.vertexPos;

	Vector3 c0 = pointOnPlane - triangle.v0.vertexPos;
	Vector3 c1 = pointOnPlane - triangle.v1.vertexPos;
	Vector3 c2 = pointOnPlane - triangle.v2.vertexPos;

	bool isPtInsideTriangle = Vector3::dot(planeNormal, Vector3::cross(edge0, c0)) >= 0 &&
		Vector3::dot(planeNormal, Vector3::cross(edge1, c1)) >= 0 &&
		Vector3::dot(planeNormal, Vector3::cross(edge2, c2)) >= 0;

	if (!isPtInsideTriangle) return false;

	//Write info only when pt is inside
	//Transform hit point to world space
	hitInfo.hitPoint = modelMatrix.TransformPoint(pointOnPlane);

	//Normal needs to use inverse transpose of the model matrix
	Matrix inverseTransposeModel;
	if (RT_FAILURE == Matrix::InverseAndTranspose(modelMatrix, inverseTransposeModel)) {
		std::cerr << "Failure during matrix inverse transpose calculation of model matrix";
		return false;
	}

	hitInfo.normal = inverseTransposeModel.TransformPoint(planeNormal);
	hitInfo.normal.normalize();
	hitInfo.distance = t;
	return true;
}

