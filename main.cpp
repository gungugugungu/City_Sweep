//
// Created by gungu on 8/19/26.
//

#define GVK_IMPLEMENTATION
#include "include/GVK-Engine/gvk.h"
#include "include/reactphysics3d/include/reactphysics3d/reactphysics3d.h"
#include <cfloat>
#include <map>
#include <glm/gtc/constants.hpp>
#include <memory>
#include <algorithm>
#include <chrono>
#include "include/SDL_mixer/include/SDL3_mixer/SDL_mixer.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

reactphysics3d::PhysicsCommon physics_common;
reactphysics3d::PhysicsWorld* world;

enum class ColliderShape {BOX, CONVEX};

enum CollisionCategory : unsigned short {
    CATEGORY_ENV = 1 << 0,
    CATEGORY_TRASH = 1 << 1,
    CATEGORY_PLAYER = 1 << 2,
};

class PhysicsObject {
public:
    gvk::MeshAsset* mesh;
    gvk::Material material;
    glm::vec3 position;
    glm::vec3 scale={1.f, 1.f, 1.f};
    glm::quat rotation;
    reactphysics3d::RigidBody* body;
    int still_frames = 0;

    PhysicsObject(gvk::MeshAsset* mesh_asset, gvk::Material mat, std::vector<gvk::Vertex> mesh_vertices, glm::vec3 pos = {0.f, 0.f, 0.f}, glm::vec3 scl = {1.f, 1.f, 1.f}, glm::quat rot = {1.f, 0.f, 0.f, 0.f}, reactphysics3d::BodyType body_type = reactphysics3d::BodyType::STATIC, ColliderShape shape = ColliderShape::BOX)
        : mesh(mesh_asset), material(mat), position(pos), scale(scl), rotation(rot)
    {
        reactphysics3d::Vector3 rp_position(position.x, position.y, position.z);
        reactphysics3d::Quaternion rp_orientation(rotation.x, rotation.y, rotation.z, rotation.w);
        reactphysics3d::Transform transform(rp_position, rp_orientation);

        body = world->createRigidBody(transform);
        body->setType(body_type);

        if (shape == ColliderShape::BOX) create_box_collider(mesh_vertices);
        else create_convex_collider(mesh_vertices);
    }

    ~PhysicsObject() {
        if (body) {
            body->removeCollider(body->getCollider(0));
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
            if (body->getLinearVelocity().lengthSquare() < 0.01f * 0.01f && body->getAngularVelocity().lengthSquare() < 0.02f * 0.02f) {
                still_frames++;
                if (still_frames > 30) body->setIsSleeping(true);
            } else {
                still_frames = 0;
            }
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
    reactphysics3d::Quaternion orientation = reactphysics3d::Quaternion::identity();
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
        body->addCollider(capsuleShape, reactphysics3d::Transform::identity());
        body->updateMassPropertiesFromColliders();

        body->setType(reactphysics3d::BodyType::DYNAMIC);
        body->enableGravity(true);
        body->setLinearDamping(0.0f);
        body->setAngularDamping(0.0f);
        body->setAngularLockAxisFactor({0.0f, 0.0f, 0.0f});
        body->setIsAllowedToSleep(false);
        body->setMass(70.0f);
        body->getCollider(0)->setCollisionCategoryBits(CATEGORY_PLAYER);
        body->getCollider(0)->setCollideWithMaskBits(CATEGORY_TRASH | CATEGORY_ENV);
    }

    void move_to(reactphysics3d::Vector3 pos) {
        position = pos;
        transform = reactphysics3d::Transform(position, orientation);
        body->setTransform(transform);
    }

    void update() {
        if (!body) return;

        auto bodyTransform = body->getTransform();
        position = bodyTransform.getPosition();
        orientation = bodyTransform.getOrientation();

        reactphysics3d::CapsuleShape* capsule = dynamic_cast<reactphysics3d::CapsuleShape*>(body->getCollider(0)->getCollisionShape());
        if (capsule) {
            float radius = capsule->getRadius();
            float height = capsule->getHeight();
            float half_height = height * 0.5f + radius;
            const float ground_check_distance = 0.15f;

            reactphysics3d::Vector3 start = bodyTransform.getPosition();
            reactphysics3d::Vector3 end = start - reactphysics3d::Vector3(0.0f, half_height + ground_check_distance, 0.0f);

            struct GroundRaycastCallback : public reactphysics3d::RaycastCallback {
                bool hit = false;
                reactphysics3d::decimal notifyRaycastHit(const reactphysics3d::RaycastInfo&) override {
                    hit = true;
                    return static_cast<rp3d::decimal>(0.0f);
                }
            };

            GroundRaycastCallback callback;
            reactphysics3d::Ray ray(start, end);
            world->raycast(ray, &callback);
            on_ground = callback.hit;
        }
    }

    void move(glm::vec3 direction) {
        if (!can_move || !body) return;

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
        body->setLinearVelocity(reactphysics3d::Vector3(world_dir.x, current_vel.y, world_dir.z));
    }

    void jump() {
        if (!can_jump || !on_ground || !body) return;

        reactphysics3d::Vector3 gravity = world->getGravity();
        float g = -gravity.y;
        float jump_velocity = std::sqrt(2.0f * jump_height * g);

        auto current_vel = body->getLinearVelocity();
        body->setLinearVelocity(reactphysics3d::Vector3(current_vel.x, jump_velocity, current_vel.z));
    }
};

class FPSController : public CharacterController {
public:
    float mouse_sensitivity = 0.005f;
    float relative_camera_height = 1.6f;

    void update_input(std::map<SDL_Keycode, bool>& inputs, float mouse_dx, float mouse_dy) {
        update();

        float pitch = asinf(gvk::camera.direction.y);
        float yaw = atan2f(-gvk::camera.direction.z, gvk::camera.direction.x);

        yaw -= mouse_dx * mouse_sensitivity;
        pitch -= mouse_dy * mouse_sensitivity;

        const float pitch_limit = glm::pi<float>() / 2.0f - 0.01f;
        pitch = glm::clamp(pitch, -pitch_limit, pitch_limit);

        yaw = std::fmod(yaw, 2.0f * glm::pi<float>());
        if (yaw < 0.0f) yaw += 2.0f * glm::pi<float>();

        orientation = reactphysics3d::Quaternion::fromEulerAngles(0.0f, yaw, 0.0f);

        {
            auto t = body->getTransform();
            t.setOrientation(orientation);
            body->setTransform(t);
        }

        glm::vec3 direction{0.0f};
        if (inputs[SDLK_W]) direction.x += 1.0f;
        if (inputs[SDLK_S]) direction.x -= 1.0f;
        if (inputs[SDLK_A]) direction.z -= 1.0f;
        if (inputs[SDLK_D]) direction.z += 1.0f;
        move(direction);

        if (inputs.count(SDLK_SPACE) && inputs.at(SDLK_SPACE) && on_ground) {
            jump();
        }

        bool is_crouching = inputs.count(SDLK_C) && inputs.at(SDLK_C);
        speed = (can_crouch && is_crouching) ? 5.0f : 10.0f;

        const auto& p = body->getTransform().getPosition();
        gvk::camera.position = {p.x, p.y + relative_camera_height, p.z};

        glm::vec3 cam_dir;
        cam_dir.x = cosf(-yaw) * cosf(pitch);
        cam_dir.y = sinf(pitch);
        cam_dir.z = sinf(-yaw) * cosf(pitch);
        gvk::camera.direction = glm::normalize(cam_dir);
    }
};

class UIButton {
public:
    gvk::Surface surf;
    glm::vec2 pos;
    std::function<void()> on_click_callback;

    void draw() {
        gvk::display.draw(surf, pos);
    }

    void update(SDL_Event* event) {
        if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
            if (event->button.button == SDL_BUTTON_LEFT && event->button.x >= pos.x && event->button.x <= pos.x + surf.pixels[0].size() && event->button.y >= pos.y && event->button.y <= pos.y + surf.pixels.size()) {
                if (on_click_callback) {
                    on_click_callback();
                }
            }
        }
    }
};

enum MenuPages {
    HELP,
    UPGRADE,
    STORAGE_UPGRADE,
    SETTINGS
};

enum UpgradePages {
    HAND,
    BROOM,
    VACUUM
};

enum Tools {
    TOOLHAND,
    TOOLBROOM,
    TOOLVACUUM
};

struct RaycastReturns {
    PhysicsObject* object;
    float distance;
    glm::vec3 hit_position;
};

optional<RaycastReturns> raycast(const vector<std::unique_ptr<PhysicsObject>>& objs, glm::vec3 start, glm::vec3 end) {
    rp3d::Ray ray{{start.x, start.y, start.z}, {end.x, end.y, end.z}, 1.f};

    PhysicsObject* closest_obj = nullptr;
    float closest_fraction = 1.f;
    glm::vec3 closest_hit_pos;

    for (auto& obj : objs) {
        if (!obj || !obj->body) continue;

        reactphysics3d::RaycastInfo info;
        if (obj->body->raycast(ray, info)) {
            if (info.hitFraction < closest_fraction) {
                closest_fraction = info.hitFraction;
                closest_obj = obj.get();
                closest_hit_pos = {info.worldPoint.x, info.worldPoint.y, info.worldPoint.z};
            }
        }
    }

    if (closest_obj) {
        float dist = closest_fraction * glm::length(end - start);
        return RaycastReturns{closest_obj, dist, closest_hit_pos};
    }

    return nullopt;
}

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

vector<std::unique_ptr<PhysicsObject>> phys_objs;

vector<PhysicsObject*> load_scene_colliders(gvk::GLTFReturns* scene) {
    vector<PhysicsObject*> objs;
    for (auto& m : scene->meshes) {
        auto obj = std::make_unique<PhysicsObject>(&m.mesh, m.material, m.mesh.vertices, m.position, m.scale, m.rot);
        objs.push_back(obj.get());
        phys_objs.push_back(std::move(obj));
    }
    return objs;
}

// -------------------- INIT --------------------
int main() {
    // engine
    relative_gvk_path = "../include/GVK-Engine";
    gvk::init();
    gvk::clear_color = {0.05f, 0.05f, 0.05f, 1.f};
    int max_fps = 120;
    gvk::_material_default_sampler = gvk::_default_sampler_nearest;

    // audio init
    if (!MIX_Init()) {
        SDL_Log("Mix init failed: %s", SDL_GetError());
        return 1;
    }
    MIX_Mixer* mixer = MIX_CreateMixerDevice(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, nullptr);
    if (!mixer) {
        SDL_Log("Failed creating mixer device: %s", SDL_GetError());
        return 1;
    }
    MIX_Track* music_track = MIX_CreateTrack(mixer);
    SDL_PropertiesID options = SDL_CreateProperties();
    SDL_SetNumberProperty(options, MIX_PROP_PLAY_LOOPS_NUMBER, -1);

    SDL_SetWindowTitle(gvk::window, "Town Sweep");

    gvk::main_post_processing_stack.ao_radius = 2.f;
    gvk::main_post_processing_stack.ao_bias = 0.008f;
    gvk::main_post_processing_stack.ao_samples = 8;
    gvk::main_post_processing_stack.vignette_strength = 0.5f;
    gvk::main_post_processing_stack.bloom_filter_threshold = 0.75f;
    gvk::main_post_processing_stack.bloom_blur_passes = 16;
    gvk::main_post_processing_stack.bloom_intensity = 1.f;
    gvk::main_post_processing_stack.pixelation_enabled = true;
    gvk::main_post_processing_stack.pixelation_size = 4;
    gvk::main_post_processing_stack.tonemap_values.op = 0;
    gvk::main_post_processing_stack.fog_distance = 1000.f;
    gvk::main_post_processing_stack.fog_intensity = 0.7f;

    // sounds
    MIX_Audio* music = MIX_LoadAudio(mixer, "../sounds/soundtrack.mp3", false);
    MIX_SetTrackAudio(music_track, music);
    MIX_PlayTrack(music_track, options);
    SDL_DestroyProperties(options);

    // fonts
    stbtt_fontinfo font;
    stbtt_fontinfo font_thin;
    gvk::load_font(&font, "../fonts/font.ttf");
    gvk::load_font(&font_thin, "../fonts/thin font.ttf");

    // physics
    rp3d::PhysicsWorld::WorldSettings _world_settings;
    _world_settings.gravity = rp3d::Vector3(0.f, -9.81f, 0.f);
    _world_settings.defaultVelocitySolverNbIterations = 5;
    world = physics_common.createPhysicsWorld(_world_settings);
    const float time_step = 1.f / 60.f; // physics fps (basically)
    float accumulator = 0.f;

    // input
    map<SDL_Keycode, bool> key_inputs;
    glm::vec2 mouse_motion_relative;
    bool mouse_locked = true;

    // textures
    gvk::load_skybox("../textures/skybox.png");
    gvk::Surface image_trashbag;
    image_trashbag.load_from_file("../textures/trashbag.png");
    gvk::Surface image_ui_background;
    image_ui_background.load_from_file("../textures/ui background.png");
    gvk::Surface image_ui_help_background;
    image_ui_help_background.load_from_file("../textures/ui help background.png");
    gvk::Surface image_ui_upgrade_background;
    image_ui_upgrade_background.load_from_file("../textures/ui upgrade background.png");
    gvk::Surface image_ui_su_background;
    image_ui_su_background.load_from_file("../textures/ui storage upgrade background.png");

    // env loading
    gvk::GLTFReturns env = gvk::load_gltf_scene("../scenes/env.glb").value();
    load_scene_lights(&env);
    vector<PhysicsObject*> env_POs = load_scene_colliders(&env);
    int fulltrashcount = 0;
    for (auto po : env_POs) {
        if (po->mesh->name.contains("pickuptrash")) { // trash
            po->body->setType(rp3d::BodyType::DYNAMIC);
            po->body->setMass(5.f);
            po->body->setLinearDamping(5.f);
            po->body->getCollider(0)->setCollisionCategoryBits(CATEGORY_TRASH);
            po->body->getCollider(0)->setCollideWithMaskBits(CATEGORY_ENV | CATEGORY_PLAYER);
            fulltrashcount++;
        } else {
            if (po->mesh->name.contains("diner")) {
                po->body->removeCollider(po->body->getCollider(0));
                po->create_convex_collider(po->mesh->vertices);
            }
            po->body->setType(rp3d::BodyType::KINEMATIC);
            po->body->getCollider(0)->setCollisionCategoryBits(CATEGORY_ENV);
            po->body->getCollider(0)->setCollideWithMaskBits(CATEGORY_TRASH | CATEGORY_PLAYER);
        }
    }
    cout << "trash count: " << fulltrashcount << endl;

    // player
    FPSController player;
    player.initalize(0.5f, 2.f);
    player.mouse_sensitivity = 0.01f;
    player.move_to({-15.f, 2.5f, 0.f});
    player.can_jump = false;
    bool mouse_left = false;
    bool mouse_right = false;
    float money = 0;
    float trash_price = 0.5f;
    Tools current_tool = TOOLHAND;
    vector<Tools> available_tools = {TOOLHAND};
    int current_tool_idx = 0;
    float tool_animation_frametime = 0.1f;
    float frame_accum = 0.f;
    int current_tool_anim_frame = 0;

    // trash data
    struct {
        int currently_stored = 0;
        int max_storable = 10;
    } trash;

    // upgrades
    function<void()> upgrade_1_buy = [&]{};
    function<void()> upgrade_2_buy = [&]{};
    function<void()> upgrade_3_buy = [&]{};
    float upgrade_1_completion = 0.f;
    float upgrade_2_completion = 0.f;
    float upgrade_3_completion = 0.f;
    float upgrade_1_price = 0.f;
    float upgrade_2_price = 0.f;
    float upgrade_3_price = 0.f;
    string upgrade_1_name;
    string upgrade_2_name;
    string upgrade_3_name;

    float timer = 0.f;

    // hand
    struct {
        float time_between_pickups = 0.5f;
        int upgrade_1_progress = 0;
        int upgrade_1_max = 10;
        float upgrade_1_decrease_by = 0.05f;
        float upgrade_1_price = 3.f;

        int upgrade_2_progress = 0;
        int upgrade_2_max = 4;
        float pickup_radius = 0.f;
        float upgrade_2_increase_by = 0.3f;
        int max_items_to_pick_up = 6;
        float upgrade_2_price = 4.f;

        bool upgrade_3_bought = false;
        float upgrade_3_price = 12.f;

        vector<gvk::Surface> idle_anim;
        vector<gvk::Surface> click_anim;
        bool anim_clicking;
    } hand;

    hand.idle_anim.resize(4); hand.click_anim.resize(4);
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/hand_idle"<<i+1<<".png";
        hand.idle_anim[i].load_from_file(ss.str());
    }
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/hand_click"<<i+1<<".png";
        hand.click_anim[i].load_from_file(ss.str());
    }

    function<void()> hand_upgrade_1_buy = [&] {
        if (hand.upgrade_1_progress < hand.upgrade_1_max && money>=upgrade_1_price) {
            hand.time_between_pickups -= hand.upgrade_1_decrease_by;
            hand.upgrade_1_progress++;
            upgrade_1_completion = static_cast<float>(hand.upgrade_1_progress)/static_cast<float>(hand.upgrade_1_max);
            money -= upgrade_1_price;
        }
    };
    function<void()> hand_upgrade_2_buy = [&] {
        if (hand.upgrade_2_progress < hand.upgrade_2_max && money>=upgrade_2_price) {
            hand.pickup_radius += hand.upgrade_2_increase_by;
            hand.upgrade_2_progress++;
            upgrade_2_completion = static_cast<float>(hand.upgrade_2_progress)/static_cast<float>(hand.upgrade_2_max);
            money -= upgrade_2_price;
        }
    };
    function<void()> hand_upgrade_3_buy = [&] {
        if (money>=upgrade_3_price) {
            hand.upgrade_3_bought = true;
            upgrade_3_completion = 1.f;
            money -= upgrade_3_price;
        }
    };

    // broom
    struct {
        bool unlocked = false;
        float price=50.f;
        float sweep_radius = 1.5f;
        float sweep_strength = 3000.f;
        float time=0.6f;

        int upgrade_1_progress = 0;
        int upgrade_1_max = 4;
        float upgrade_1_increase_by = 0.3f;
        float upgrade_1_price = 3.f;

        int upgrade_2_progress = 0;
        int upgrade_2_max = 5;
        float upgrade_2_increase_by = 0.25f;
        float upgrade_2_price = 3.2f;

        int upgrade_3_progress = 0;
        int upgrade_3_max = 3;
        float upgrade_3_decrease_by = 0.2f;
        float upgrade_3_price = 5.0f;

        vector<gvk::Surface> idle_anim;
        vector<gvk::Surface> click_anim;
        bool anim_clicking=false;
    } broom;

    broom.idle_anim.resize(4); broom.click_anim.resize(4);
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/broom_idle"<<i+1<<".png";
        broom.idle_anim[i].load_from_file(ss.str());
    }
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/broom_click"<<i+1<<".png";
        broom.click_anim[i].load_from_file(ss.str());
    }

    function<void()> broom_upgrade_1_buy = [&] {
        if (broom.upgrade_1_progress < broom.upgrade_1_max && money>=upgrade_1_price) {
            broom.sweep_radius += broom.upgrade_1_increase_by;
            broom.upgrade_1_progress++;
            upgrade_1_completion = static_cast<float>(broom.upgrade_1_progress)/static_cast<float>(broom.upgrade_1_max);
            money -= upgrade_1_price;
        }
    };
    function<void()> broom_upgrade_2_buy = [&] {
        if (broom.upgrade_2_progress < broom.upgrade_2_max && money>=upgrade_2_price) {
            broom.sweep_strength += broom.upgrade_2_increase_by;
            broom.upgrade_2_progress++;
            upgrade_2_completion = static_cast<float>(broom.upgrade_2_progress)/static_cast<float>(broom.upgrade_2_max);
            money -= upgrade_2_price;
        }
    };
    function<void()> broom_upgrade_3_buy = [&] {
        if (broom.upgrade_3_progress < broom.upgrade_3_max && money>=upgrade_3_price) {
            broom.time -= broom.upgrade_3_decrease_by;
            broom.upgrade_3_progress++;
            upgrade_3_completion = static_cast<float>(broom.upgrade_3_progress)/static_cast<float>(broom.upgrade_3_max);
            money -= upgrade_3_price;
        }
    };

    // vacuum
    struct {
        bool unlocked = false;
        float price=200.f;
        float suck_distance = 10.0f;
        float suck_strength = 500.f;
        float suck_radius = 1.5f;
        float pickup_radius = 3.5f;

        int upgrade_1_progress = 0;
        int upgrade_1_max = 4;
        float upgrade_1_increase_by = 300.f;
        float upgrade_1_price = 6.0f;

        int upgrade_2_progress = 0;
        int upgrade_2_max = 6;
        float upgrade_2_increase_by = 1.0f;
        float upgrade_2_price = 3.0f;

        int upgrade_3_progress = 0;
        int upgrade_3_max = 3;
        float upgrade_3_increase_by = 0.2f;
        float upgrade_3_price = 10.f;

        vector<gvk::Surface> idle_anim;
        vector<gvk::Surface> click_anim;
        bool anim_clicking=false;
    } vacuum;

    vacuum.idle_anim.resize(4); vacuum.click_anim.resize(4);
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/vacuum_idle"<<i+1<<".png";
        vacuum.idle_anim[i].load_from_file(ss.str());
    }
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss<<"../textures/vacuum_click"<<i+1<<".png";
        vacuum.click_anim[i].load_from_file(ss.str());
    }

    function<void()> vacuum_upgrade_1_buy = [&] {
        if (vacuum.upgrade_1_progress < vacuum.upgrade_1_max && money>=upgrade_1_price) {
            vacuum.suck_strength += vacuum.upgrade_1_increase_by;
            vacuum.upgrade_1_progress++;
            upgrade_1_completion = static_cast<float>(vacuum.upgrade_1_progress)/static_cast<float>(vacuum.upgrade_1_max);
            money -= upgrade_1_price;
        }
    };
    function<void()> vacuum_upgrade_2_buy = [&] {
        if (vacuum.upgrade_2_progress < vacuum.upgrade_2_max && money>=upgrade_2_price) {
            vacuum.suck_distance += vacuum.upgrade_2_increase_by;
            vacuum.upgrade_2_progress++;
            upgrade_2_completion = static_cast<float>(vacuum.upgrade_2_progress)/static_cast<float>(vacuum.upgrade_2_max);
            money -= upgrade_2_price;
        }
    };
    function<void()> vacuum_upgrade_3_buy = [&] {
        if (vacuum.upgrade_3_progress < vacuum.upgrade_3_max && money>=upgrade_3_price) {
            vacuum.suck_radius += vacuum.upgrade_3_increase_by;
            vacuum.upgrade_3_progress++;
            upgrade_3_completion = static_cast<float>(vacuum.upgrade_3_progress)/static_cast<float>(vacuum.upgrade_3_max);
            money -= upgrade_3_price;
        }
    };

    // storage upgrades
    bool upgrade_backpack_bought=false;
    bool upgrade_trashbag_bought=false;
    bool upgrade_wheelbarrow_bought=false;
    bool upgrade_trashcan_bought=false;
    float upgrade_backpack_price = 10.f;
    int upgrade_backpack_amount = 25;
    float upgrade_trashbag_price = 25.f;
    int upgrade_trashbag_amount = 50;
    float upgrade_wheelbarrow_price = 50.f;
    int upgrade_wheelbarrow_amount = 100;
    float upgrade_trashcan_price = 75.f;
    int upgrade_trashcan_amount = 125;
    function<void()> buy_backpack_upgrade = [&] {
        if (!upgrade_backpack_bought && money >= upgrade_backpack_price) {
            upgrade_backpack_bought = true;
            trash.max_storable += upgrade_backpack_amount;
            money -= upgrade_backpack_price;
        }
    };
    function<void()> buy_trashbag_upgrade = [&] {
        if (!upgrade_trashbag_bought && money >= upgrade_trashbag_price) {
            upgrade_trashbag_bought = true;
            trash.max_storable += upgrade_trashbag_amount;
            money -= upgrade_trashbag_price;
        }
    };
    function<void()> buy_wheelbarrow_upgrade = [&] {
        if (!upgrade_wheelbarrow_bought && money >= upgrade_wheelbarrow_price) {
            upgrade_wheelbarrow_bought = true;
            trash.max_storable += upgrade_wheelbarrow_amount;
            money -= upgrade_wheelbarrow_price;
        }
    };
    function<void()> buy_trashcan_upgrade = [&] {
        if (!upgrade_trashcan_bought && money >= upgrade_trashcan_price) {
            upgrade_trashcan_bought = true;
            trash.max_storable += upgrade_trashcan_amount;
            money -= upgrade_trashcan_price;
        }
    };

    // UI
    vector<UIButton*> buttons_to_update;
    vector<UIButton*> buttons_upgrade_menu;
    vector<UIButton*> buttons_storage_upgrade_menu;
    MenuPages current_menu_page = HELP;
    UpgradePages current_upgrade_page = HAND;
    bool pause_menu_open = false;
    function<void()> open_pause_menu = [&]{ pause_menu_open = true; mouse_locked = false; };
    function<void()> close_pause_menu = [&]{ pause_menu_open = false; mouse_locked = true; };
    UIButton button_help;
    button_help.surf.load_from_file("../textures/ui help button.png");
    button_help.pos = {272, 212};
    button_help.on_click_callback = [&] {
        current_menu_page = HELP;
    };
    buttons_to_update.push_back(&button_help);

    UIButton button_upgrades;
    button_upgrades.surf.load_from_file("../textures/ui upgrade button.png");
    button_upgrades.pos = {272, 324};
    button_upgrades.on_click_callback = [&] {
        current_menu_page = UPGRADE;
    };
    buttons_to_update.push_back(&button_upgrades);

    UIButton button_storage_upgrades;
    button_storage_upgrades.surf.load_from_file("../textures/ui storage upgrade button.png");
    button_storage_upgrades.pos = {272, 436};
    button_storage_upgrades.on_click_callback = [&] {
        current_menu_page = STORAGE_UPGRADE;
    };
    buttons_to_update.push_back(&button_storage_upgrades);

    UIButton button_settings;
    button_settings.surf.load_from_file("../textures/ui settings button.png");
    button_settings.pos = {272, 652};
    button_settings.on_click_callback = [&] {
        current_menu_page = SETTINGS;
    };
    buttons_to_update.push_back(&button_settings);

    UIButton button_quit;
    button_quit.surf.load_from_file("../textures/ui quit button.png");
    button_quit.pos = {272, 764};
    buttons_to_update.push_back(&button_quit);

    UIButton button_close;
    button_close.surf.load_from_file("../textures/ui close button.png");
    button_close.pos = {1600, 212};
    button_close.on_click_callback = [&] {
        close_pause_menu();
    };
    buttons_to_update.push_back(&button_close);

    UIButton button_upgrade_hand;
    button_upgrade_hand.surf.load_from_file("../textures/ui upgrade hand button.png");
    button_upgrade_hand.pos = {492, 248};
    button_upgrade_hand.on_click_callback = [&] {
        current_upgrade_page = HAND;
        upgrade_1_buy = hand_upgrade_1_buy;
        upgrade_2_buy = hand_upgrade_2_buy;
        upgrade_3_buy = hand_upgrade_3_buy;
        upgrade_1_name = "Pickup speed";
        upgrade_2_name = "Palm size";
        upgrade_3_name = "Hold to pick up";
        upgrade_1_completion = static_cast<float>(hand.upgrade_1_progress)/static_cast<float>(hand.upgrade_1_max);
        upgrade_2_completion = static_cast<float>(hand.upgrade_2_progress)/static_cast<float>(hand.upgrade_2_max);
        upgrade_3_completion = (hand.upgrade_3_bought) ? 1.f : 0.f;
        upgrade_1_price = hand.upgrade_1_price;
        upgrade_2_price = hand.upgrade_2_price;
        upgrade_3_price = hand.upgrade_3_price;
    };
    button_upgrade_hand.on_click_callback();
    buttons_upgrade_menu.push_back(&button_upgrade_hand);

    UIButton button_upgrade_broom;
    button_upgrade_broom.surf.load_from_file("../textures/ui upgrade broom button.png");
    button_upgrade_broom.pos = {844, 248};
    button_upgrade_broom.on_click_callback = [&] {
        if (!broom.unlocked && money>=broom.price) {
            broom.unlocked = true;
            current_tool = TOOLBROOM;
            available_tools.push_back(TOOLBROOM);
            current_tool_idx = available_tools.size()-1;
            money -= broom.price;
        }
        if (broom.unlocked) {
            current_upgrade_page = BROOM;
            upgrade_1_buy = broom_upgrade_1_buy;
            upgrade_2_buy = broom_upgrade_2_buy;
            upgrade_3_buy = broom_upgrade_3_buy;
            upgrade_1_name = "Radius";
            upgrade_2_name = "Strength";
            upgrade_3_name = "Speed";
            upgrade_1_completion = static_cast<float>(broom.upgrade_1_progress)/static_cast<float>(broom.upgrade_1_max);
            upgrade_2_completion = static_cast<float>(broom.upgrade_2_progress)/static_cast<float>(broom.upgrade_2_max);
            upgrade_3_completion = static_cast<float>(broom.upgrade_3_progress)/static_cast<float>(broom.upgrade_3_max);
            upgrade_1_price = broom.upgrade_1_price;
            upgrade_2_price = broom.upgrade_2_price;
            upgrade_3_price = broom.upgrade_3_price;
        }
    };
    buttons_upgrade_menu.push_back(&button_upgrade_broom);

    UIButton button_upgrade_vacuum;
    button_upgrade_vacuum.surf.load_from_file("../textures/ui upgrade vacuum button.png");
    button_upgrade_vacuum.pos = {1196, 248};
    button_upgrade_vacuum.on_click_callback = [&] {
        if (!vacuum.unlocked && money >= vacuum.price) {
            vacuum.unlocked = true;
            current_tool = TOOLVACUUM;
            available_tools.push_back(TOOLVACUUM);
            current_tool_idx = available_tools.size()-1;
            money -= vacuum.price;
        }
        if (vacuum.unlocked) {
            current_upgrade_page = VACUUM;
            upgrade_1_buy = vacuum_upgrade_1_buy;
            upgrade_2_buy = vacuum_upgrade_2_buy;
            upgrade_3_buy = vacuum_upgrade_3_buy;
            upgrade_1_name = "Power";
            upgrade_2_name = "Distance";
            upgrade_3_name = "Spread";
            upgrade_1_completion = static_cast<float>(vacuum.upgrade_1_progress)/static_cast<float>(vacuum.upgrade_1_max);
            upgrade_2_completion = static_cast<float>(vacuum.upgrade_2_progress)/static_cast<float>(vacuum.upgrade_2_max);
            upgrade_3_completion = static_cast<float>(vacuum.upgrade_3_progress)/static_cast<float>(vacuum.upgrade_3_max);
            upgrade_1_price = vacuum.upgrade_1_price;
            upgrade_2_price = vacuum.upgrade_2_price;
            upgrade_3_price = vacuum.upgrade_3_price;
        }
    };
    buttons_upgrade_menu.push_back(&button_upgrade_vacuum);

    UIButton button_upgrade_frame_1;
    button_upgrade_frame_1.surf.load_from_file("../textures/ui upgrade frame button.png");
    button_upgrade_frame_1.pos = {492, 376};
    button_upgrade_frame_1.on_click_callback = [&] {
        if (upgrade_1_buy) upgrade_1_buy();
    };
    buttons_upgrade_menu.push_back(&button_upgrade_frame_1);

    UIButton button_upgrade_frame_2;
    button_upgrade_frame_2.surf.load_from_file("../textures/ui upgrade frame button.png");
    button_upgrade_frame_2.pos = {492, 540};
    button_upgrade_frame_2.on_click_callback = [&] {
        if (upgrade_2_buy) upgrade_2_buy();
    };
    buttons_upgrade_menu.push_back(&button_upgrade_frame_2);

    UIButton button_upgrade_frame_3;
    button_upgrade_frame_3.surf.load_from_file("../textures/ui upgrade frame button.png");
    button_upgrade_frame_3.pos = {492, 704};
    button_upgrade_frame_3.on_click_callback = [&] {
        if (upgrade_3_buy) upgrade_3_buy();
    };
    buttons_upgrade_menu.push_back(&button_upgrade_frame_3);

    UIButton button_su_trashcan;
    button_su_trashcan.surf.load_from_file("../textures/ui su trashcan buy.png");
    button_su_trashcan.pos = {448, 252};
    button_su_trashcan.on_click_callback = buy_trashcan_upgrade;
    buttons_storage_upgrade_menu.push_back(&button_su_trashcan);

    UIButton button_su_wheelbarrow;
    button_su_wheelbarrow.surf.load_from_file("../textures/ui su wheelbarrow buy.png");
    button_su_wheelbarrow.pos = {772, 292};
    button_su_wheelbarrow.on_click_callback = buy_wheelbarrow_upgrade;
    buttons_storage_upgrade_menu.push_back(&button_su_wheelbarrow);

    UIButton button_su_trashbag;
    button_su_trashbag.surf.load_from_file("../textures/ui su trashbag buy.png");
    button_su_trashbag.pos = {1068, 332};
    button_su_trashbag.on_click_callback = buy_trashbag_upgrade;
    buttons_storage_upgrade_menu.push_back(&button_su_trashbag);

    UIButton button_su_backpack;
    button_su_backpack.surf.load_from_file("../textures/ui su backpack buy.png");
    button_su_backpack.pos = {1364, 368};
    button_su_backpack.on_click_callback = buy_backpack_upgrade;
    buttons_storage_upgrade_menu.push_back(&button_su_backpack);

    function<void()> load_storage_upgrade_images = [&] {
        (upgrade_backpack_bought) ? button_su_backpack.surf.load_from_file("../textures/ui su backpack bought.png") : button_su_backpack.surf.load_from_file("../textures/ui su backpack buy.png");
        (upgrade_trashbag_bought) ? button_su_trashbag.surf.load_from_file("../textures/ui su trashbag bought.png") : button_su_trashbag.surf.load_from_file("../textures/ui su trashbag buy.png");
        (upgrade_wheelbarrow_bought) ? button_su_wheelbarrow.surf.load_from_file("../textures/ui su wheelbarrow bought.png") : button_su_wheelbarrow.surf.load_from_file("../textures/ui su wheelbarrow buy.png");
        (upgrade_trashcan_bought) ? button_su_trashcan.surf.load_from_file("../textures/ui su trashcan bought.png") : button_su_trashcan.surf.load_from_file("../textures/ui su trashcan buy.png");
    };

    function<void()> load_tool_button_images = [&] {
        (broom.unlocked) ? button_upgrade_broom.surf.load_from_file("../textures/ui upgrade broom button.png") : button_upgrade_broom.surf.load_from_file("../textures/ui upgrade buy button.png");
        (vacuum.unlocked) ? button_upgrade_vacuum.surf.load_from_file("../textures/ui upgrade vacuum button.png") : button_upgrade_vacuum.surf.load_from_file("../textures/ui upgrade buy button.png");
    };

    // picking up trash
    function<void(RaycastReturns hit)> pickup_trash = [&](RaycastReturns hit) {
        if (trash.currently_stored < trash.max_storable && timer <= 0.f) {
            // pickup radius
            if (hand.upgrade_2_progress > 0) {
                glm::vec3 ogpos = {hit.object->body->getTransform().getPosition().x, hit.object->body->getTransform().getPosition().y, hit.object->body->getTransform().getPosition().z};
                glm::vec3 pos;
                vector<PhysicsObject*> close_objs;
                for (auto& o : phys_objs) {
                    pos = {o->body->getTransform().getPosition().x, o->body->getTransform().getPosition().y, o->body->getTransform().getPosition().z};
                    if (glm::distance(ogpos, pos) <= hand.pickup_radius && o.get()->mesh->name.contains("pickuptrash") && o.get() != hit.object) {
                        close_objs.push_back(o.get());
                    }
                }
                for (int i = 0; i<hand.max_items_to_pick_up; i++) {
                    if (trash.currently_stored >= trash.max_storable) break;
                    if (i < static_cast<int>(close_objs.size())) {
                        auto it = std::find_if(phys_objs.begin(), phys_objs.end(), [&](const auto& up){ return up.get() == close_objs[i]; });
                        if (it != phys_objs.end()) {
                            phys_objs.erase(it);
                        }
                        trash.currently_stored++;
                    } else break;
                }
            }

            if (trash.currently_stored < trash.max_storable) {
                auto it = std::find_if(phys_objs.begin(), phys_objs.end(), [&](const auto& up){ return up.get() == hit.object; });
                if (it != phys_objs.end()) {
                    phys_objs.erase(it);
                }
                trash.currently_stored++;
                timer = hand.time_between_pickups;
            }

            // anim
            current_tool_anim_frame = 0;
            hand.anim_clicking = true;
        }
    };

    Uint64 last_time = SDL_GetTicks();
    float mouse_x, mouse_y = 0.f;
    bool running = true;
    button_quit.on_click_callback = [&] {
        running = false;
    };

// -------------------- FRAME --------------------
    while (running) {
        // general shit
        Uint64 now = SDL_GetTicks();
        float dt = static_cast<float>(now - last_time) / 1000.f;
        accumulator += dt;
        frame_accum += dt;
        last_time = now;
        int w_width, w_height;
        SDL_GetWindowSize(gvk::window, &w_width, &w_height);
        if (pause_menu_open) {
            gvk::display.clear(w_width, w_height, {0.f, 0.f, 0.f, 0.5f});
        } else {
            gvk::display.clear(w_width, w_height, {1, 1, 1, 0});
        }

        timer = clamp(timer-dt, 0.f, 10.f);

        mouse_motion_relative.x = 0.f;
        mouse_motion_relative.y = 0.f;

        // mouse locking
        if (mouse_locked) {
            SDL_HideCursor();
            SDL_SetWindowRelativeMouseMode(gvk::window, true);
        } else {
            SDL_ShowCursor();
            SDL_SetWindowRelativeMouseMode(gvk::window, false);
        }

        // sdl events
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (pause_menu_open) {
                for (auto& b : buttons_to_update) {
                    b->update(&e);
                }

                if (current_menu_page == UPGRADE) {
                    for (auto& b : buttons_upgrade_menu) {
                        b->update(&e);
                    }
                } else if (current_menu_page == STORAGE_UPGRADE) {
                    for (auto& b : buttons_storage_upgrade_menu) {
                        b->update(&e);
                    }
                }
            }

            if (e.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (e.type == SDL_EVENT_KEY_DOWN) { // KEY DOWN
                key_inputs[e.key.key] = true;

                if (e.key.key == SDLK_ESCAPE) {
                    if (pause_menu_open) close_pause_menu(); else open_pause_menu();
                }
            }
            if (e.type == SDL_EVENT_KEY_UP) { // KEY UP
                key_inputs[e.key.key] = false;
            }
            if (e.type == SDL_EVENT_MOUSE_MOTION) { // MOUSE MOVED
                mouse_motion_relative.x = e.motion.xrel;
                mouse_motion_relative.y = e.motion.yrel;
                mouse_x = e.motion.x;
                mouse_y = e.motion.y;
            }
            if (e.type == SDL_EVENT_MOUSE_WHEEL) { // MOUSE SCROLL
                current_tool_idx += e.wheel.integer_y;
                if (current_tool_idx < 0) current_tool_idx = available_tools.size()-1;
                if (current_tool_idx > available_tools.size()-1) current_tool_idx = 0;
                current_tool = available_tools[current_tool_idx];
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) { // MOUSE PRESSED
                if (e.button.button == SDL_BUTTON_LEFT) {
                    mouse_left = true;

                    if (pause_menu_open && current_menu_page==STORAGE_UPGRADE) load_storage_upgrade_images();
                    if (pause_menu_open && current_menu_page==UPGRADE) load_tool_button_images();

                    if (!pause_menu_open) {
                        auto hit_opt = raycast(phys_objs, gvk::camera.position, gvk::camera.position + gvk::camera.direction * 5.f);

                        if (hit_opt.has_value()) {
                            RaycastReturns& hit = *hit_opt;

                            // -------------------- RAYCASTS --------------------
                            if (hit.object) {
                                // picking up trash
                                if (hit.object->mesh->name.contains("pickuptrash") && current_tool == TOOLHAND) {
                                    pickup_trash(hit);
                                }

                                // brooming
                                else if (current_tool == TOOLBROOM) {
                                    if (timer <= 0) {
                                        glm::vec3 pos;
                                        for (auto& o : phys_objs) {
                                            pos = {o->body->getTransform().getPosition().x, o->body->getTransform().getPosition().y, o->body->getTransform().getPosition().z};
                                            if (glm::distance(hit.hit_position, pos) <= broom.sweep_radius && o.get()->mesh->name.contains("pickuptrash")) {
                                                o->body->applyWorldForceAtCenterOfMass({gvk::camera.direction.x*broom.sweep_strength, 0, gvk::camera.direction.z*broom.sweep_strength});
                                                broom.anim_clicking = true;
                                                current_tool_anim_frame = 0;
                                            }
                                        }
                                        timer = broom.time;
                                    }
                                }

                                // trash bin
                                else if (hit.object->mesh->name.contains("trashbin")) {
                                    money += trash.currently_stored*trash_price;
                                    trash.currently_stored = 0;
                                }
                            }
                        }
                    }
                }
                if (e.button.button == SDL_BUTTON_RIGHT) mouse_right=true;
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (e.button.button == SDL_BUTTON_LEFT) mouse_left=false;
                if (e.button.button == SDL_BUTTON_RIGHT) mouse_right=false;
            }
        }
        // -------------------- POST-EVENT UPDATES --------------------
        // player stuffity stuff
        if (mouse_locked) player.update_input(key_inputs, mouse_motion_relative.x, mouse_motion_relative.y);
        // hold pickup
        if (!pause_menu_open && mouse_left && hand.upgrade_3_bought && current_tool == TOOLHAND) {
            auto hit_opt = raycast(phys_objs, gvk::camera.position, gvk::camera.position + gvk::camera.direction * 5.f);

            if (hit_opt.has_value()) {
                RaycastReturns& hit = *hit_opt;
                if (hit.object) {
                    if (hit.object->mesh->name.contains("pickuptrash")) {
                        pickup_trash(hit);
                    }
                }
            }
        }

        // vacuum
        if (!pause_menu_open && mouse_left && current_tool == TOOLVACUUM) {
            auto hit_opt = raycast(phys_objs, gvk::camera.position, gvk::camera.position + gvk::camera.direction * vacuum.suck_distance);

            // pulling in objects
            if (hit_opt.has_value()) {
                RaycastReturns& hit = *hit_opt;
                if (hit.object) {
                    glm::vec3 pos;

                    for (auto& o : phys_objs) {
                        if (o != nullptr) {
                            pos = {o->body->getTransform().getPosition().x, o->body->getTransform().getPosition().y, o->body->getTransform().getPosition().z};
                            if (glm::distance(hit.hit_position, pos) <= vacuum.suck_radius && o.get()->mesh->name.contains("pickuptrash")) {
                                o.get()->body->applyWorldForceAtCenterOfMass({-gvk::camera.direction.x*vacuum.suck_strength, 0, -gvk::camera.direction.z*vacuum.suck_strength});
                                vacuum.anim_clicking = true;
                            }
                        }
                    }
                }
            }

            // picking up them bitches
            for (auto& o : phys_objs) {
                if (o != nullptr) {
                    glm::vec3 pos = {o->body->getTransform().getPosition().x, o->body->getTransform().getPosition().y, o->body->getTransform().getPosition().z};
                    if (glm::distance({player.body->getTransform().getPosition().x, player.body->getTransform().getPosition().y, player.body->getTransform().getPosition().z}, pos) <= vacuum.pickup_radius && trash.currently_stored < trash.max_storable && o.get()->mesh->name.contains("pickuptrash")) {
                        auto it = std::find_if(phys_objs.begin(), phys_objs.end(), [&](const auto& up){ return up.get() == o.get();});
                        if (it != phys_objs.end()) {
                            phys_objs.erase(it);
                        }
                        trash.currently_stored++;
                    }
                }
            }
        }

        if (!mouse_left && current_tool == TOOLVACUUM) vacuum.anim_clicking = false;

        // physics
        if (accumulator>=time_step) {
            world->update(time_step);
            accumulator -= time_step;
        }

        // imgui
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        // imgui goes here
        ImGui::Render();

// -------------------- 2D RENDERING --------------------
        // current tool
        if (frame_accum >= tool_animation_frametime) {
            if (current_tool_anim_frame < 3) { current_tool_anim_frame++; } else { current_tool_anim_frame = 0; }
            frame_accum -= tool_animation_frametime;
        }
        if (current_tool == TOOLHAND) {
            if (hand.anim_clicking) {
                gvk::display.draw(hand.click_anim[current_tool_anim_frame], {0,0});
                if (current_tool_anim_frame == 3) hand.anim_clicking = false;
            } else {
                gvk::display.draw(hand.idle_anim[current_tool_anim_frame], {0,0});
            }
        }
        if (current_tool == TOOLBROOM) {
            if (broom.anim_clicking) {
                gvk::display.draw(broom.click_anim[current_tool_anim_frame], {0,0});
                if (current_tool_anim_frame == 3) broom.anim_clicking = false;
            } else {
                gvk::display.draw(broom.idle_anim[current_tool_anim_frame], {0,0});
            }
        }
        if (current_tool == TOOLVACUUM) {
            if (vacuum.anim_clicking) {
                gvk::display.draw(vacuum.click_anim[current_tool_anim_frame], {0,0});
            } else {
                gvk::display.draw(vacuum.idle_anim[current_tool_anim_frame], {0,0});
            }
        }

        // trash amount display
        gvk::display.draw(image_trashbag, {1624, 768});
        glm::vec2 counter_text_size = gvk::get_text_size(&font, to_string(trash.currently_stored), 128.f);
        gvk::display.draw_text(&font, to_string(trash.currently_stored), {1624+116-(counter_text_size.x*0.5f), 768+128-(counter_text_size.y*0.5f)}, 128.f, {0.396078431372549f, 0.45098039215686275f, 0.5725490196078431f, 1.f});

        // crosshair
        gvk::display.draw_rect(6, 6, {w_width*0.5f-3, w_height*0.5f-3}, {1, 1, 1, 0.5f});

// -------------------- PAUSE MENU --------------------
        // money
        stringstream money_text;
        money_text<<money<<"$";
        glm::vec2 money_text_size = gvk::get_text_size(&font, money_text.str(), 96.f);
        gvk::display.draw_text(&font, money_text.str(), {w_width-16-static_cast<int>(money_text_size.x), 16}, 96.f, {1, 0.784313725, 0.145098039, 1});

        if (pause_menu_open) {
            gvk::display.draw(image_ui_background, {384, 188});

            // help menu
            if (current_menu_page == MenuPages::HELP) {
                gvk::display.draw(image_ui_help_background, {384, 188});
            }

            // upgrade menu
            else if (current_menu_page == MenuPages::UPGRADE) {
                gvk::display.draw(image_ui_upgrade_background, {384, 188});

                for (auto& b : buttons_upgrade_menu) {
                    b->draw();
                }

                // broom
                if (!broom.unlocked) {
                    stringstream broom_price_text;
                    broom_price_text<<broom.price<<" $";
                    glm::vec2 broom_price_text_size = gvk::get_text_size(&font, broom_price_text.str(), 48.f);
                    gvk::display.draw_text(&font, broom_price_text.str(), {844+136-broom_price_text_size.x*0.5f, 248+40-broom_price_text_size.y*0.5f}, 48.f, {0.352941176, 0.77254902, 0.309803922, 1});
                }

                // vacuum
                if (!vacuum.unlocked) {
                    stringstream vacuum_price_text;
                    vacuum_price_text<<vacuum.price<<" $";
                    glm::vec2 vacuum_price_text_size = gvk::get_text_size(&font, vacuum_price_text.str(), 48.f);
                    gvk::display.draw_text(&font, vacuum_price_text.str(), {1196+136-vacuum_price_text_size.x*0.5f, 248+40-vacuum_price_text_size.y*0.5f}, 48.f, {0.352941176, 0.77254902, 0.309803922, 1});
                }

                // upgrade completion bars
                gvk::display.draw_rect(1096, 112, {508, 392}, {0.9647058823529412f, 0.5058823529411764f, 0.5294117647058824f, 1.f});
                if (upgrade_1_completion > 0.f) gvk::display.draw_rect(static_cast<int>(1096.f*upgrade_1_completion), 112, {508, 392}, {0.35294117647058826f, 0.7725490196078432f, 0.30980392156862746f, 1.f});
                gvk::display.draw_rect(1096, 112, {508, 556}, {0.9647058823529412f, 0.5058823529411764f, 0.5294117647058824f, 1.f});
                if (upgrade_2_completion > 0.f) gvk::display.draw_rect(static_cast<int>(1096.f*upgrade_2_completion), 112, {508, 556}, {0.35294117647058826f, 0.7725490196078432f, 0.30980392156862746f, 1.f});
                gvk::display.draw_rect(1096, 112, {508, 720}, {0.9647058823529412f, 0.5058823529411764f, 0.5294117647058824f, 1.f});
                if (upgrade_3_completion > 0.f) gvk::display.draw_rect(static_cast<int>(1096.f*upgrade_3_completion), 112, {508, 720}, {0.35294117647058826f, 0.7725490196078432f, 0.30980392156862746f, 1.f});

                // upgrade names
                if (upgrade_1_name != "") {
                    glm::vec2 text_size = gvk::get_text_size(&font, upgrade_1_name, 72);
                    gvk::display.draw_text(&font, upgrade_1_name, {492+564-text_size.x*0.5, 376+72-text_size.y*0.5}, 72, {0.152941176, 0.152941176, 0.152941176, 1});
                }
                if (upgrade_2_name != "") {
                    glm::vec2 text_size = gvk::get_text_size(&font, upgrade_2_name, 72);
                    gvk::display.draw_text(&font, upgrade_2_name, {492+564-text_size.x*0.5, 540+72-text_size.y*0.5}, 72, {0.152941176, 0.152941176, 0.152941176, 1});
                }
                if (upgrade_3_name != "") {
                    glm::vec2 text_size = gvk::get_text_size(&font, upgrade_3_name, 72);
                    gvk::display.draw_text(&font, upgrade_3_name, {492+564-text_size.x*0.5, 704+72-text_size.y*0.5}, 72, {0.152941176, 0.152941176, 0.152941176, 1});
                }

                // upgrade prices
                if (mouse_x >= button_upgrade_frame_1.pos.x && mouse_x <= button_upgrade_frame_1.pos.x + button_upgrade_frame_1.surf.pixels[0].size() && mouse_y >= button_upgrade_frame_1.pos.y && mouse_y <= button_upgrade_frame_1.pos.y + button_upgrade_frame_1.surf.pixels.size()) {
                    stringstream ss;
                    ss<<upgrade_1_price<<"$";
                    if (money>=upgrade_1_price) {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {1, 0.784313725, 0.145098039, 1});
                    } else {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {0.917647059, 0.196078431, 0.196078431, 1});
                    }
                }

                if (mouse_x >= button_upgrade_frame_2.pos.x && mouse_x <= button_upgrade_frame_2.pos.x + button_upgrade_frame_2.surf.pixels[0].size() && mouse_y >= button_upgrade_frame_2.pos.y && mouse_y <= button_upgrade_frame_2.pos.y + button_upgrade_frame_2.surf.pixels.size()) {
                    stringstream ss;
                    ss<<upgrade_2_price<<"$";
                    if (money>=upgrade_2_price) {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {1, 0.784313725, 0.145098039, 1});
                    } else {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {0.917647059, 0.196078431, 0.196078431, 1});
                    }
                }
                if (mouse_x >= button_upgrade_frame_3.pos.x && mouse_x <= button_upgrade_frame_3.pos.x + button_upgrade_frame_3.surf.pixels[0].size() && mouse_y >= button_upgrade_frame_3.pos.y && mouse_y <= button_upgrade_frame_3.pos.y + button_upgrade_frame_3.surf.pixels.size()) {
                    stringstream ss;
                    ss<<upgrade_3_price<<"$";
                    if (money>=upgrade_3_price) {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {1, 0.784313725, 0.145098039, 1});
                    } else {
                        gvk::display.draw_text(&font, ss.str(), {mouse_x+8, mouse_y+8}, 64.f, {0.917647059, 0.196078431, 0.196078431, 1});
                    }
                }
            }

            // storage upgrade menu
            else if (current_menu_page == MenuPages::STORAGE_UPGRADE) {
                gvk::display.draw(image_ui_su_background, {384, 188});

                for (auto& b : buttons_storage_upgrade_menu) {
                    b->draw();
                }
            }

            // settings menu
            else if (current_menu_page == MenuPages::SETTINGS) {
                gvk::display.draw_text(&font, "SETTINGS MENU", {384+64, 192+64}, 72, {1, 1, 1, 1});
            }

            for (auto& b : buttons_to_update) {
                b->draw();
            }
        }

// -------------------- RENDERING --------------------
        for (auto& po : phys_objs) {
            po->update();
            po->render();
        }

        gvk::display.refresh();
        gvk::draw();
    }

// -------------------- QUIT --------------------

    vkDeviceWaitIdle(gvk::_vk_device);
    physics_common.destroyPhysicsWorld(world);

    gvk::quit();
    return 0;
}