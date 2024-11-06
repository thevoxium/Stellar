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

// Body Definition

struct AABB {
  vec2 min, max;
};

struct Circle {
  vec2 center;
  double radius;
};

struct Body {
  vec2 position, velocity, acceleration, forceAccumulator;
  double mass, inverseMass, damping, angle;
  bool isStatic, isActive;

  enum ShapeType {
    SHAPE_CIRCLE,
    SHAPE_AABB,
  } shapeType;

  union {
    Circle circle;
    AABB aabb;
  } shape;
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

  if (body.shapeType == Body::SHAPE_CIRCLE) {
    body.shape.circle.center = body.position;
  } else {
    vec2 halfSize =
        vec2_mul(vec2_sub(body.shape.aabb.max, body.shape.aabb.min), 0.5);
    body.shape.aabb.min = vec2_sub(body.position, halfSize);
    body.shape.aabb.max = vec2_add(body.position, halfSize);
  }

  clearForces(body);
}

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

double calculateArea(const Body &body) {
  if (body.shapeType == Body::SHAPE_CIRCLE) {
    return M_PI * body.shape.circle.radius * body.shape.circle.radius;
  } else {
    vec2 size = vec2_sub(body.shape.aabb.max, body.shape.aabb.min);
    return size.x * size.y;
  }
  return 0.0;
}

double calculateMOI(const Body &body) {
  if (body.shapeType == Body::SHAPE_CIRCLE) {
    return 0.5 * body.mass * body.shape.circle.radius *
           body.shape.circle.radius;
  } else {
    vec2 size = vec2_sub(body.shape.aabb.max, body.shape.aabb.min);
    return body.mass * (size.x * size.x + size.y * size.y) / 12;
  }
}

AABB getShapeBoundingBox(const Body &body) {
  if (body.shapeType == Body::SHAPE_CIRCLE) {
    return {{body.position.x - body.shape.circle.radius,
             body.position.y - body.shape.circle.radius},
            {body.position.x + body.shape.circle.radius,
             body.position.y + body.shape.circle.radius}};
  }
  return body.shape.aabb; // AABB already represents its bounding box
}

bool containsPoint(const Body &body, const vec2 &point) {
  if (body.shapeType == Body::SHAPE_CIRCLE) {
    vec2 diff = vec2_sub(point, body.position);
    return vec2_length_squared(diff) <=
           body.shape.circle.radius * body.shape.circle.radius;
  } else {
    return point.x >= body.shape.aabb.min.x &&
           point.x <= body.shape.aabb.max.x &&
           point.y >= body.shape.aabb.min.y && point.y <= body.shape.aabb.max.y;
  }
  return false;
}

vec2 getFurthestPoint(const Body &body, vec2 direction) {
  vec2_normalize(direction);
  if (body.shapeType == Body::SHAPE_CIRCLE) {
    return vec2_add(body.position,
                    vec2_mul(direction, body.shape.circle.radius));
  } else {
    vec2 result = body.position;
    result.x += (direction.x >= 0) ? (body.shape.aabb.max.x - body.position.x)
                                   : (body.shape.aabb.min.x - body.position.x);
    result.y += (direction.y >= 0) ? (body.shape.aabb.max.y - body.position.y)
                                   : (body.shape.aabb.min.y - body.position.y);
    return result;
  }
  return body.position;
}

int addCircle(World &world, vec2 position, double radius, double mass) {
  Body body{};
  body.position = position;
  body.velocity = {0, 0};
  body.acceleration = {0, 0};
  body.mass = mass;
  body.inverseMass = mass > 0.0 ? 1.0 / mass : 0.0;
  body.damping = 0.99;
  body.isStatic = mass <= 0;
  body.isActive = true;
  body.angle = 0.0;
  body.forceAccumulator = {0, 0};

  body.shapeType = Body::SHAPE_CIRCLE;
  body.shape.circle = {position, radius};

  world.bodies.push_back(body);
  return world.bodies.size() - 1;
}
int addBox(World &world, vec2 position, vec2 size, double mass) {
  Body body{};
  body.position = position;
  body.velocity = {0, 0};
  body.acceleration = {0, 0};
  body.mass = mass;
  body.inverseMass = mass > 0 ? 1.0 / mass : 0;
  body.damping = 0.99;
  body.isStatic = mass <= 0;
  body.isActive = true;
  body.angle = 0;

  body.shapeType = Body::SHAPE_AABB;
  body.shape.aabb = {{position.x - size.x / 2, position.y - size.y / 2},
                     {position.x + size.x / 2, position.y + size.y / 2}};

  world.bodies.push_back(body);
  return world.bodies.size() - 1;
}

/**/
/*int main() {*/
/*  World world;*/
/*  initWorld(world);*/
/**/
/*  int circleId = addCircle(world, {0, 10}, 1.0, 1.0);*/
/*  int boxId = addBox(world, {5, 10}, {2, 2}, 2.0);*/
/**/
/*  Body &circle = world.bodies[circleId];*/
/*  std::cout << "Circle area: " << calculateArea(circle) << "\n";*/
/*  std::cout << "Circle MOI: " << calculateMOI(circle) << "\n";*/
/**/
/*  double fixedTimeStep = 1.0 / 60.0; // 60 Hz simulation*/
/*  for (int i = 0; i < 100; i++) {*/
/*    stepWorld(world, fixedTimeStep);*/
/**/
/*    std::cout << "Step " << i << ":\n";*/
/*    std::cout << "Circle position: " << world.bodies[circleId].position.x*/
/*              << ", " << world.bodies[circleId].position.y << "\n";*/
/*    std::cout << "Box position: " << world.bodies[boxId].position.x << ", "*/
/*              << world.bodies[boxId].position.y << "\n\n";*/
/*  }*/
/**/
/*  return 0;*/
/*}*/
