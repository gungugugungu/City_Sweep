//
// Created by gungu on 8/19/26.
//

#define GVK_IMPLEMENTATION
#include "include/GVK-Engine/gvk.h"
#include "include/reactphysics3d/include/reactphysics3d/reactphysics3d.h"
#include <cfloat>
#include <map>
#include <glm/gtc/constants.hpp>

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

    PhysicsObject(gvk::MeshAsset* mesh_asset, gvk::Material mat, std::vector<gvk::Vertex> mesh_vertices, glm::vec3 pos = {0.f, 0.f, 0.f}, glm::vec3 scl = {1.f, 1.f, 1.f}, glm::quat rot = {1.f, 0.f, 0.f, 0.f}, reactphysics3d::BodyType body_type = reactphysics3d::BodyType::STATIC)
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

class CharacterController {
public:
    reactphysics3d::Vector3 position;
    reactphysics3d::Quaternion orientation;
    reactphysics3d::Transform transform;
    reactphysics3d::RigidBody* body = nullptr;
    float speed = 10.0f;
    float jump_height = 3.0f;
    bool on_ground = false;
    bool can_jump = true;
    bool can_move = true;
    bool can_crouch = true;
    bool can_crouch_jump = true;

    void initalize(float collider_width, float collider_height) {
        transform = reactphysics3d::Transform::identity();
        body = world->createRigidBody(transform);
        reactphysics3d::CapsuleShape* capsuleShape = physics_common.createCapsuleShape(collider_width, collider_height);
        body->addCollider(capsuleShape, transform);
        body->setType(reactphysics3d::BodyType::DYNAMIC);
        body->enableGravity(true);
        body->setLinearDamping(0.0f);
        body->setAngularDamping(0.0f);
        body->setAngularLockAxisFactor({0.0f,0.0f,0.0f});
        body->setIsAllowedToSleep(false);
    }

    void move_to(reactphysics3d::Vector3 position) {
        transform = reactphysics3d::Transform(position, orientation);
        body->setTransform(transform);
    }

    void update() {
        auto transform = body->getTransform();
        reactphysics3d::CapsuleShape* capsule = dynamic_cast<reactphysics3d::CapsuleShape*>(body->getCollider(0)->getCollisionShape());
        if (capsule) {
            float radius = capsule->getRadius();
            float height = capsule->getHeight();
            float half_height = (height*0.5f)+radius;
            const float ground_check_distance = 0.1f;

            reactphysics3d::Vector3 start = transform.getPosition();
            reactphysics3d::Vector3 end = start - reactphysics3d::Vector3(0.0f, half_height+ground_check_distance, 0.0f);

            struct GroundRaycastCallback : public reactphysics3d::RaycastCallback {
                bool hit = false;
                reactphysics3d::decimal notifyRaycastHit(const reactphysics3d::RaycastInfo& raycastinfo) override {
                    hit = true;
                    return 0.0f;
                }
            };

            GroundRaycastCallback callback;
            reactphysics3d::Ray ray(start, end);
            world->raycast(ray, &callback);
            on_ground = callback.hit;
        }
    }

    void move(glm::vec3 direction) {
        if (!can_move) return;

        reactphysics3d::Vector3 local_dir(direction.x, 0.0f, direction.z);
        if (local_dir.lengthSquare() < 0.001f) {
            auto current_vel = body->getLinearVelocity();
            body->setLinearVelocity(reactphysics3d::Vector3(0, current_vel.y, 0));
            return;
        }

        local_dir.normalize();
        reactphysics3d::Vector3 world_dir = orientation * local_dir;

        world_dir *= speed;

        auto current_vel = body->getLinearVelocity();
        reactphysics3d::Vector3 new_vel(world_dir.x, current_vel.y, world_dir.z);
        body->setLinearVelocity(new_vel);
    }

    void jump() {
        if (!can_jump) return;

        reactphysics3d::Vector3 gravity = world->getGravity();
        float g = -gravity.y;
        float jump_velocity = std::sqrt(2.0f*jump_height*g);

        auto current_vel = body->getLinearVelocity();
        reactphysics3d::Vector3 new_vel(current_vel.x, jump_velocity, current_vel.z);
        body->setLinearVelocity(new_vel);
    }
};

class FPSController : public CharacterController {
public:
    float mouse_sensitivity = 0.005f;
    float relative_camera_height;
    void update_input(std::map<SDL_Keycode, bool>& inputs, float mouse_dx, float mouse_dy) {
        update();

        float pitch = asinf(gvk::camera.direction.y);
        float yaw = atan2f(-gvk::camera.direction.z, gvk::camera.direction.x);

        yaw -= mouse_dx * mouse_sensitivity;
        pitch -= mouse_dy * mouse_sensitivity;

        float pitch_limit = glm::pi<float>() / 2.0f - 0.01f;
        if (pitch > pitch_limit) pitch = pitch_limit;
        if (pitch < -pitch_limit) pitch = -pitch_limit;

        yaw = std::fmod(yaw, 2.0f * glm::pi<float>());
        if (yaw < 0.0f) yaw += 2.0f * glm::pi<float>();

        orientation = reactphysics3d::Quaternion::fromEulerAngles(0.0f, yaw, 0.0f);

        glm::vec3 direction = {0.0f, 0.0f, 0.0f};
        if (inputs[SDLK_W] == true) direction.x += 1.0f;
        if (inputs[SDLK_S] == true) direction.x -= 1.0f;
        if (inputs[SDLK_A] == true) direction.z -= 1.0f;
        if (inputs[SDLK_D] == true) direction.z += 1.0f;
        move(direction);

        if (inputs.find(SDLK_SPACE) != inputs.end() && inputs.at(SDLK_SPACE) && on_ground) {
            jump();
        }

        bool is_crouching = (inputs.find(SDLK_C) != inputs.end() && inputs.at(SDLK_C));
        if (can_crouch && is_crouching) {
            speed = 5.0f;
        } else {
            speed = 10.0f;
        }

        gvk::camera.position.x = body->getTransform().getPosition().x;
        gvk::camera.position.y = body->getTransform().getPosition().y + relative_camera_height;
        gvk::camera.position.z = body->getTransform().getPosition().z;

        direction.x = cosf(-yaw) * cosf(pitch);
        direction.y = sinf(pitch);
        direction.z = sinf(-yaw) * cosf(pitch);
        gvk::camera.direction = glm::normalize(direction);
    }
};

void load_scene_lights(gvk::GLTFReturns* scene) {
    gvk::directional_light.intensity = scene->dir_light.intensity;
    gvk::directional_light.direction = scene->dir_light.direction;
    gvk::directional_light.color = scene->dir_light.color;
    for (auto pl : scene->point_lights) {
        gvk::point_lights.push_back(pl);
    }
    for (auto sl : scene->spot_lights) {
        gvk::spot_lights.push_back(sl);
    }
}

vector<PhysicsObject*> phys_objs;

vector<PhysicsObject*> load_scene_colliders(gvk::GLTFReturns* scene) {
    vector<PhysicsObject*> objs;
    for (auto& m : scene->meshes) {
        PhysicsObject* obj = new PhysicsObject(&m.mesh, m.material, m.vertices, m.position, m.scale, m.rot);
        objs.push_back(obj);
        phys_objs.push_back(obj);
    }
    return objs;
}

// -------------------- INIT --------------------
int main() {
    relative_gvk_path = "../include/GVK-Engine";
    gvk::init();
    gvk::clear_color = {0.05f, 0.05f, 0.05f, 1.f};

    world = physics_common.createPhysicsWorld();
    map<SDL_Keycode, bool> key_inputs;
    glm::vec2 mouse_motion_relative;
    bool mouse_locked = true;

    gvk::load_skybox("../include/GVK-Engine/textures/skyboxes/generic clouds.png");

    // dev env loading
    gvk::GLTFReturns dev_env = gvk::load_gltf_scene("../scenes/devenv.glb").value();
    load_scene_lights(&dev_env);
    vector<PhysicsObject*> dev_env_POs = load_scene_colliders(&dev_env);
    for (auto po : dev_env_POs) {
        po->body->setType(rp3d::BodyType::KINEMATIC);
    }
    dev_env_POs[0]->body->setType(rp3d::BodyType::KINEMATIC);

    FPSController player;
    player.initalize(0.5f, 2.f);
    player.mouse_sensitivity = 0.03f;
    player.position.y = 5.f;

    gvk::camera.position.y = 2.f;

    Uint64 last_time = SDL_GetTicks();
    bool running = true;

    // -------------------- FRAME --------------------
    while (running) {
        // general shit
        Uint64 now = SDL_GetTicks();
        float dt = (float)(now - last_time) / 1000.f;
        last_time = now;
        int w_width, w_height;
        SDL_GetWindowSize(gvk::window, &w_width, &w_height);

        mouse_motion_relative.x = 0.f;
        mouse_motion_relative.y = 0.f;

        // mouse locking
        if (mouse_locked) {
            SDL_HideCursor();
            SDL_WarpMouseInWindow(gvk::window, w_width*0.5f, w_height*0.5f);
        } else {
            SDL_ShowCursor();
        }

        // sdl events
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (e.type == SDL_EVENT_KEY_DOWN) {
                key_inputs[e.key.key] = true;
            }
            if (e.type == SDL_EVENT_KEY_UP) {
                key_inputs[e.key.key] = false;
            }
            if (e.type == SDL_EVENT_MOUSE_MOTION) {
                mouse_motion_relative.x = e.motion.xrel;
                mouse_motion_relative.y = e.motion.yrel;
            }
        }
        //player.update_input(key_inputs, mouse_motion_relative.x, -mouse_motion_relative.y);

        world->update(dt);

        // imgui
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        // imgui goes here
        ImGui::Render();

        // -------------------- RENDERING --------------------
        for (auto& po : phys_objs) {
            po->update();
            po->render();
        }

        gvk::draw();
    }

    // -------------------- QUIT --------------------

    vkDeviceWaitIdle(gvk::_vk_device);

    gvk::quit();
    return 0;
}