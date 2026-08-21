//
// Created by gungu on 8/19/26.
//

#define GVK_IMPLEMENTATION
#include "include/GVK-Engine/gvk.h"
#include "include/reactphysics3d/include/reactphysics3d/reactphysics3d.h"
#include <cfloat>

reactphysics3d::PhysicsCommon physics_common;
reactphysics3d::PhysicsWorld* world;

class PhysicsObject {
public:
    gvk::MeshAsset* mesh;
    gvk::Material material;
    glm::vec3 position;
    glm::vec3 scale;
    glm::quat rotation;
    reactphysics3d::RigidBody* body;

    PhysicsObject(gvk::MeshAsset* mesh_asset, gvk::Material mat, std::vector<gvk::Vertex> mesh_vertices, glm::vec3 pos = {0.f, 0.f, 0.f}, glm::vec3 scl = {1.f, 1.f, 1.f}, glm::quat rot = {1.f, 0.f, 0.f, 0.f}, reactphysics3d::BodyType body_type = reactphysics3d::BodyType::DYNAMIC)
        : mesh(mesh_asset), material(mat), position(pos), scale(scl), rotation(rot)
    {
        reactphysics3d::Vector3 rp_position(position.x, position.y, position.z);
        reactphysics3d::Quaternion rp_orientation(rotation.x, rotation.y, rotation.z, rotation.w);
        reactphysics3d::Transform transform(rp_position, rp_orientation);

        body = world->createRigidBody(transform);
        body->setType(body_type);

        create_convex_collider(mesh_vertices);
    }

    ~PhysicsObject() {
        if (body) {
            world->destroyRigidBody(body);
        }
    }

    void create_box_collider(std::vector<gvk::Vertex> vertices) {
        if (vertices.empty()) {
            return;
        }

        float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
        float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;

        for (const auto& vtx : vertices) {
            float x = vtx.position.x * scale.x;
            float y = vtx.position.y * scale.y;
            float z = vtx.position.z * scale.z;

            minX = std::min(minX, x);
            minY = std::min(minY, y);
            minZ = std::min(minZ, z);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
            maxZ = std::max(maxZ, z);
        }

        reactphysics3d::Vector3 halfExtents((maxX - minX) * 0.5f, (maxY - minY) * 0.5f, (maxZ - minZ) * 0.5f);

        halfExtents.x = std::max(halfExtents.x, 0.01f);
        halfExtents.y = std::max(halfExtents.y, 0.01f);
        halfExtents.z = std::max(halfExtents.z, 0.01f);

        reactphysics3d::BoxShape* boxShape = physics_common.createBoxShape(halfExtents);
        reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();
        body->addCollider(boxShape, colliderTransform);
    }

    void create_convex_collider(std::vector<gvk::Vertex> vertices) {
        if (vertices.size() < 4) {
            cout << "Not enough data for convex mesh, using box collider" << endl;
            create_box_collider(vertices);
            return;
        }

        std::vector<reactphysics3d::Vector3> rp_vertices;
        rp_vertices.reserve(vertices.size());

        for (const auto& vtx : vertices) {
            rp_vertices.emplace_back(vtx.position.x * scale.x, vtx.position.y * scale.y, vtx.position.z * scale.z);
        }

        reactphysics3d::Vector3* vertex_data = rp_vertices.data();

        reactphysics3d::VertexArray vertex_array = reactphysics3d::VertexArray(vertex_data, sizeof(reactphysics3d::Vector3), rp_vertices.size(), reactphysics3d::VertexArray::DataType::VERTEX_FLOAT_TYPE);
        std::vector<reactphysics3d::Message> messages;
        reactphysics3d::ConvexMesh* convexMesh = physics_common.createConvexMesh(vertex_array, messages);

        if (convexMesh) {
            reactphysics3d::ConvexMeshShape* shape = physics_common.createConvexMeshShape(convexMesh);
            reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();
            body->addCollider(shape, colliderTransform);
        } else {
            cout << "Failed to create convex mesh. Messages:" << endl;
            for (const auto& message : messages) {
                cout << "  " << message.text << endl;
            }
            create_box_collider(vertices);
        }
    }

        void create_concave_collider(std::vector<gvk::Vertex> vertices, std::vector<uint32_t> indices) {
        if (vertices.empty() || indices.empty() || indices.size() % 3 != 0) {
            create_box_collider(vertices);
            return;
        }

        if (body->getType() != reactphysics3d::BodyType::STATIC) {
            cout << "warning: concave colliders should only be used on static bodies" << endl;
        }

        std::vector<reactphysics3d::Vector3> rp_vertices;
        rp_vertices.reserve(vertices.size());

        for (const auto& vtx : vertices) {
            rp_vertices.emplace_back(vtx.position.x * scale.x, vtx.position.y * scale.y, vtx.position.z * scale.z);
        }

        reactphysics3d::TriangleVertexArray triangle_array(
            (uint32_t)rp_vertices.size(), rp_vertices.data(), sizeof(reactphysics3d::Vector3),
            (uint32_t)(indices.size() / 3), indices.data(), 3 * sizeof(uint32_t),
            reactphysics3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
            reactphysics3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE
        );

        std::vector<reactphysics3d::Message> messages;
        reactphysics3d::TriangleMesh* triangleMesh = physics_common.createTriangleMesh(triangle_array, messages);

        if (triangleMesh) {
            reactphysics3d::ConcaveMeshShape* shape = physics_common.createConcaveMeshShape(triangleMesh);
            reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();
            body->addCollider(shape, colliderTransform);
        } else {
            cout << "Failed to create triangle mesh. Messages:" << endl;
            for (const auto& message : messages) {
                cout << "  " << message.text << endl;
            }
            create_box_collider(vertices);
        }
    }

    void update() {
        if (body != nullptr) {
            reactphysics3d::Vector3 p = body->getTransform().getPosition();
            reactphysics3d::Quaternion q = body->getTransform().getOrientation();
            position.x = p.x;
            position.y = p.y;
            position.z = p.z;
            rotation.x = q.x;
            rotation.y = q.y;
            rotation.z = q.z;
            rotation.w = q.w;
        }
    }

    void render() {
        gvk::draw_mesh(mesh, material, position, scale, rotation);
    }
};

int main() {
    relative_gvk_path = "../include/GVK-Engine";
    gvk::init();

    world = physics_common.createPhysicsWorld();

    gvk::clear_color = {0.05f, 0.05f, 0.05f, 1.f};

    gvk::load_skybox("../include/GVK-Engine/textures/skyboxes/generic clouds.png");

    Uint64 last_time = SDL_GetTicks();

    bool running = true;
    while (running) {
        Uint64 now = SDL_GetTicks();
        float dt = (float)(now - last_time) / 1000.f;
        last_time = now;
        int w_width, w_height;
        SDL_GetWindowSize(gvk::window, &w_width, &w_height);

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                running = false;
            }
        }

        world->update(dt);

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        // imgui goes here
        ImGui::Render();

        gvk::draw();
    }

    vkDeviceWaitIdle(gvk::_vk_device);

    gvk::quit();
    return 0;
}