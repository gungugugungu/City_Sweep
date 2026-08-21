//
// Created by gungu on 8/19/26.
//

#define GVK_IMPLEMENTATION
#include "include/GVK-Engine/gvk.h"
#include "include/reactphysics3d/include/reactphysics3d/reactphysics3d.h"

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

    PhysicsObject(gvk::MeshAsset* mesh_asset, gvk::Material mat, glm::vec3 pos = {0.f, 0.f, 0.f}, glm::vec3 scl = {1.f, 1.f, 1.f}, glm::quat rot = {1.f, 0.f, 0.f, 0.f}, reactphysics3d::BodyType body_type = reactphysics3d::BodyType::DYNAMIC) : mesh(mesh_asset), material(mat), position(pos), scale(scl), rotation(rot)
    {
        reactphysics3d::Vector3 rp_position(position.x, position.y, position.z);
        reactphysics3d::Quaternion rp_orientation(rotation.x, rotation.y, rotation.z, rotation.w);
        reactphysics3d::Transform transform(rp_position, rp_orientation);

        body = world->createRigidBody(transform);
        body->setType(body_type);

        create_box_collider();
    }

    ~PhysicsObject() {
        if (body) {
            world->destroyRigidBody(body);
        }
    }

    void create_box_collider() {
        if (!mesh) {
            return;
        }

        glm::vec3 min = mesh->AABB_min * scale;
        glm::vec3 max = mesh->AABB_max * scale;

        reactphysics3d::Vector3 halfExtents((max.x - min.x) * 0.5f, (max.y - min.y) * 0.5f, (max.z - min.z) * 0.5f);

        halfExtents.x = std::max(halfExtents.x, 0.01f);
        halfExtents.y = std::max(halfExtents.y, 0.01f);
        halfExtents.z = std::max(halfExtents.z, 0.01f);

        reactphysics3d::BoxShape* boxShape = physics_common.createBoxShape(halfExtents);
        reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();
        body->addCollider(boxShape, colliderTransform);
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