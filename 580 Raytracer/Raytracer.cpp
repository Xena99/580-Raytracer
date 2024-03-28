#include "Raytracer.h"
#include <iostream>
#include "ExternalPlugins/json.hpp"
#include <fstream>

//For timing render duration
#include <chrono> 
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
	Vector3 pointOnPlane = ray.origin + (ray.direction * t);

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


//////Helper Functions//////
int Raytracer::LoadMesh(const std::string meshName) {
	auto it = mScene->meshMap.find(meshName);
	if (it != mScene->meshMap.end()) {
		std::cout << "Mesh map already contains " << meshName << ". Skipped loading" << std::endl;
		return RT_SUCCESS;
	}

	std::ifstream file(ASSETS_PATH + meshName + ".json");
	if (!file.is_open()) {
		std::cout << "File with name " << ASSETS_PATH << meshName << ".json" << " could not be found";
		return RT_FAILURE;
	}

	nlohmann::json jsonData;
	file >> jsonData;

	Mesh* mesh = new Mesh();
	for (const auto& item : jsonData["data"]) {
		Triangle triangle;
		// Parse vertices
		for (int i = 0; i < 3; ++i) {
			std::string vertexKey = "v" + std::to_string(i);
			Vector3 pos = { item[vertexKey]["v"][0], item[vertexKey]["v"][1], item[vertexKey]["v"][2] };
			Vector3 norm = { item[vertexKey]["n"][0], item[vertexKey]["n"][1], item[vertexKey]["n"][2] };
			Vector2 tex = { item[vertexKey]["t"][0], item[vertexKey]["t"][1] };

			Vertex vertex;
			vertex.vertexPos = pos;
			vertex.vertexNormal = norm;
			vertex.texture = tex;

			switch (i) {
			case 0: triangle.v0 = vertex; break;
			case 1: triangle.v1 = vertex; break;
			case 2: triangle.v2 = vertex; break;
			}
		}

		mesh->triangles.push_back(triangle);
	}

	mScene->meshMap[meshName] = mesh;
	return RT_SUCCESS;
}

int Raytracer::LoadSceneJSON(const std::string scenePath) {
	using json = nlohmann::json;
	int status = 0;
	std::ifstream file(ASSETS_PATH + scenePath); // Assuming the JSON is stored in a file named "scene.json"

	if (!file.is_open()) {
		std::cerr << "Failed to open JSON file" << " Path: " << ASSETS_PATH << scenePath << "\n";
		return RT_FAILURE;
	}

	// Parse the JSON
	json jsonData;
	try {
		file >> jsonData;
	}
	catch (const std::exception& e) {
		std::cout << "Error parsing JSON" << "\n";
		return RT_FAILURE;
	}

	try {
		mScene = new Scene();
		// Parse shapes
		if (jsonData["scene"].find("shapes") != jsonData["scene"].end()) {
			for (const auto& shapeValue : jsonData["scene"]["shapes"]) {
				Shape shape;
				shape.id = shapeValue["id"];
				shape.geometryId = shapeValue["geometry"];
				shape.notes = shapeValue["notes"];

				//Write material
				const auto& material = shapeValue["material"];
				shape.material.surfaceColor.x = material["Cs"][0];
				shape.material.surfaceColor.y = material["Cs"][1];
				shape.material.surfaceColor.z = material["Cs"][2];
				shape.material.Ka = material["Ka"];
				shape.material.Kd = material["Kd"];
				shape.material.Ks = material["Ks"];
				shape.material.specularExponet = material["n"];

				//Auto load into mesh map
				status |= LoadMesh(shape.geometryId);

				//Write transformations
				const auto& transforms = shapeValue["transforms"];
				shape.transforms.scale.x = transforms[1]["S"][0];
				shape.transforms.scale.y = transforms[1]["S"][1];
				shape.transforms.scale.z = transforms[1]["S"][2];

				//Write rotations
				if (transforms[0].find("Ry") != transforms[0].end()) {
					shape.transforms.rotation.y = transforms[0]["Ry"];
				}
				if (transforms[0].find("Rx") != transforms[0].end()) {
					shape.transforms.rotation.x = transforms[0]["Rx"];
				}
				if (transforms[0].find("Rz") != transforms[0].end()) {
					shape.transforms.rotation.z = transforms[0]["Rz"];
				}

				shape.transforms.translation.x = transforms[2]["T"][0];
				shape.transforms.translation.y = transforms[2]["T"][1];
				shape.transforms.translation.z = transforms[2]["T"][2];

				mScene->shapes.push_back(shape);
			}
		}

		// Parse camera
		if (jsonData["scene"].find("camera") != jsonData["scene"].end()) {
			const auto& cameraValue = jsonData["scene"]["camera"];
			mScene->camera.from.x = cameraValue["from"][0];
			mScene->camera.from.y = cameraValue["from"][1];
			mScene->camera.from.z = cameraValue["from"][2];
			mScene->camera.to.x = cameraValue["to"][0];
			mScene->camera.to.y = cameraValue["to"][1];
			mScene->camera.to.z = cameraValue["to"][2];
			mScene->camera.near = cameraValue["bounds"][0];
			mScene->camera.far = cameraValue["bounds"][1];
			mScene->camera.right = cameraValue["bounds"][2];
			mScene->camera.left = cameraValue["bounds"][3];
			mScene->camera.top = cameraValue["bounds"][4];
			mScene->camera.bottom = cameraValue["bounds"][5];

			mScene->camera.xRes = cameraValue["resolution"][0];
			mScene->camera.yRes = cameraValue["resolution"][1];
		}

		//Parse lights
		if (jsonData["scene"].find("lights") != jsonData["scene"].end()) {
			for (const auto& lightValue : jsonData["scene"]["lights"]) {
				Light light;

				// Common properties
				light.color = Vector3(lightValue["color"][0], lightValue["color"][1], lightValue["color"][2]);
				light.intensity = lightValue["intensity"];

				std::string typeStr = lightValue["type"];
				if (typeStr == "directional") {
					Vector3 from = Vector3(lightValue["from"][0], lightValue["from"][1], lightValue["from"][2]);
					Vector3 to = Vector3(lightValue["to"][0], lightValue["to"][1], lightValue["to"][2]);

					mScene->directional = light;
					mScene->directional.direction = to - from;
					mScene->directional.direction.normalize();
				}
				mScene->lights.push_back(light);
			}
		}
		std::cout << "Scene parsing completed!\n";
		return status;
	}
	catch (const std::exception& e) {
		std::cout << "Error parsing JSON " << e.what() << "\n";
		return RT_FAILURE;
	}
}

int main() {
	//For recording duration stats
	auto startTime = std::chrono::high_resolution_clock::now();

	//Do ray tracing
	Raytracer* rt = new Raytracer();
	rt->LoadSceneJSON("scene.json");

	auto stopTime = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
	std::cout << "\Raytracer completed in " << duration.count() << " milliseconds.\n";

	return 0;
}
