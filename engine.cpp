#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

// Vector Operations

struct vec2 {
  double x;
  double y;
};

vec2 vec2_add(const vec2 &a, const vec2 &b) {
  return vec2{a.x + b.x, a.y + b.y};
}

vec2 vec2_sub(const vec2 &a, const vec2 &b) {
  return vec2{a.x - b.x, a.y - b.y};
}

vec2 vec2_mul(const vec2 &a, double scalar) {
  return vec2{a.x * scalar, a.y * scalar};
}

double vec2_dot(const vec2 &a, const vec2 &b) { return a.x * b.x + a.y * b.y; }

double vec2_cross(const vec2 &a, const vec2 &b) {
  return a.x * b.y - b.x * a.y;
}

void vec2_normalize(vec2 &a) {
  double length = sqrt(a.x * a.x + a.y * a.y);
  if (length > 0) {
    a.x /= length;
    a.y /= length;
  }
}

double vec2_length(const vec2 &a) { return sqrt(a.x * a.x + a.y * a.y); }

double vec2_length_squared(const vec2 &a) { return a.x * a.x + a.y * a.y; }

void vec2_rotate(vec2 &a, const double &theta) {
  double original_x = a.x;
  a.x = original_x * cos(theta) - a.y * sin(theta);
  a.y = original_x * sin(theta) + a.y * cos(theta);
}

// Body Defination

struct Body {
  vec2 position, velocity, acceleration, forceAccumulator;
  double mass, inverseMass, damping;
  bool isStatic, isActive;
};

void applyForce(Body &body, const vec2 &force) {
  if (body.isStatic || !body.isActive) {
    return;
  }
  body.forceAccumulator = vec2_add(body.forceAccumulator, force);
}

void applyImpulse(Body &body, const vec2 &impulse) {
  if (body.isStatic || !body.isActive) {
    return;
  }
  vec2 deltaVelocity = vec2_mul(impulse, body.inverseMass);
  body.velocity = vec2_add(body.velocity, deltaVelocity);
}

void clearForces(Body &body) { body.forceAccumulator = {0.0, 0.0}; }

const vec2 GRAVITY{0.0, -9.81};

void applyGravity(Body &body) {
  if (body.isStatic || !body.isActive)
    return;
  vec2 gravityForce = vec2_mul(GRAVITY, body.mass);
  applyForce(body, gravityForce);
}

void integrateLinearMotion(Body &body, double dt) {
  if (body.isStatic || !body.isActive)
    return;

  if (dt <= 0.0)
    return;

  body.acceleration = vec2_mul(body.forceAccumulator, body.inverseMass);
  vec2 deltaVelocity = vec2_mul(body.acceleration, dt);
  body.velocity = vec2_add(body.velocity, deltaVelocity);
  body.velocity = vec2_mul(body.velocity, pow(body.damping, dt));
  vec2 deltaP = vec2_mul(body.velocity, dt);
  body.position = vec2_add(body.position, deltaP);
  clearForces(body);
}

struct AABB {
  vec2 min, max;
};

struct WorldConfig {
  vec2 gravity;
  double maxDeltaTime;
  AABB bounds;

  WorldConfig() {
    gravity.x = 0.0;
    gravity.y = -9.81;
    maxDeltaTime = 0.016;
    bounds.min.x = -100.0;
    bounds.min.y = -100.0;
    bounds.max.x = 100.0;
    bounds.max.y = 100.0;
  }
};

struct World {
  std::vector<Body> bodies;
  WorldConfig config;
  int activeBodyCount{0};
  bool isPaused{false};
  std::vector<int> queryResultIndices;
};

void initWorld(World &world, const WorldConfig config = WorldConfig{}) {
  world.bodies.clear();
  world.config = config;
  world.activeBodyCount = 0;
  world.isPaused = false;
  world.queryResultIndices.reserve(100);
}

void clearWorld(World &world) {
  world.bodies.clear();
  world.activeBodyCount = 0;
  world.queryResultIndices.clear();
}

int addBody(World &world, Body &body) {
  world.bodies.push_back(body);
  if (body.isActive)
    world.activeBodyCount++;
  return static_cast<int>(world.bodies.size() - 1);
}

void removeBody(World &world, int bodyIndex) {
  if (bodyIndex < 0 || bodyIndex >= world.bodies.size())
    return;
  if (world.bodies[bodyIndex].isActive) {
    world.activeBodyCount--;
  }
  if (bodyIndex < world.bodies.size() - 1) {
    world.bodies[bodyIndex] = world.bodies.back();
  }
  world.bodies.pop_back();
}

void clearAllForces(World &world) {
  for (auto body : world.bodies) {
    if (body.isActive)
      clearForces(body);
  }
}

void updatePhysics(World &world, double dt) {
  for (auto &body : world.bodies) {
    if (body.isActive) {
      applyGravity(body);
      integrateLinearMotion(body, dt);
    }
  }
}

void stepWorld(World &world, double dt) {
  if (world.isPaused)
    return;
  dt = min(dt, world.config.maxDeltaTime);
  updatePhysics(world, dt);
}

vector<int> queryBodiesInRegion(World &world, const AABB &region) {
  world.queryResultIndices.clear();
  for (size_t i = 0; i < world.bodies.size(); i++) {
    const auto &body = world.bodies[i];
    if (!body.isActive)
      continue;
    if (body.position.x >= region.min.x && body.position.x <= region.max.x &&
        body.position.y >= region.min.y && body.position.y <= region.max.y) {
      world.queryResultIndices.push_back(static_cast<int>(i));
    }
  }
  return world.queryResultIndices;
}
