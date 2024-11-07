#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>

using namespace std;

// Vector Operations

const double e = 0.2; // Restitution

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

struct GridCell {
  vector<int> bodyIndex;
};

struct Grid {
  vector<GridCell> cells;
  double cellSize;
  int numRows, numCols;
  vec2 worldMin, worldMax;
};

void updateBroadPhase(Grid &grid, const vector<Body> &bodies);
void initGrid(Grid &grid, AABB &worldBound, double cellSize);

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
  vec2 deltaP = vec2_add(vec2_mul(body.velocity, dt),
                         vec2_mul(body.acceleration, 0.5 * dt * dt));
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
  vector<Body> bodies;
  WorldConfig config;
  int activeBodyCount{0};
  bool isPaused{false};
  vector<int> queryResultIndices;
  Grid grid;
};

void initWorld(World &world, const WorldConfig config = WorldConfig{}) {
  world.bodies.clear();
  world.config = config;
  world.activeBodyCount = 0;
  world.isPaused = false;
  world.queryResultIndices.reserve(100);

  WorldConfig copyConfig = world.config;
  initGrid(world.grid, copyConfig.bounds, 10.0);
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
  updateBroadPhase(world.grid, world.bodies);
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
void initGrid(Grid &grid, AABB &worldBound, double cellSize) {
  grid.worldMin = worldBound.min;
  grid.worldMax = worldBound.max;
  grid.cellSize = cellSize;

  vec2 worldSize = vec2_sub(worldBound.max, worldBound.min);
  grid.numRows = static_cast<int>(worldSize.x / cellSize);
  grid.numCols = static_cast<int>(worldSize.y / cellSize);
  grid.cells.resize(grid.numRows * grid.numCols);
}

void clearGrid(Grid &grid) {
  for (auto &cell : grid.cells) {
    cell.bodyIndex.clear();
  }
}

int getCellIndex(const Grid &grid, int row, int col) {
  return row * grid.numCols + col;
}

void getGridCell(const Grid &grid, const vec2 &position, int &row, int &col) {
  vec2 relativePos = vec2_sub(position, grid.worldMin);
  col = static_cast<int>(relativePos.x / grid.cellSize);
  row = static_cast<int>(relativePos.y / grid.cellSize);
  col = max(0, min(col, grid.numCols - 1));
  row = max(0, min(row, grid.numRows - 1));
}

void insertBodyIntoGrid(Grid &grid, int bodyIndex, const Body &body) {
  AABB bounds = getShapeBoundingBox(body);

  int startRow, startCol, endRow, endCol;
  getGridCell(grid, bounds.min, startRow, startCol);
  getGridCell(grid, bounds.max, endRow, endCol);

  for (int row = startRow; row <= endRow; ++row) {
    for (int col = startCol; col <= endCol; ++col) {
      int cellIndex = getCellIndex(grid, row, col);
      grid.cells[cellIndex].bodyIndex.push_back(bodyIndex);
    }
  }
}

struct BroadPhaseCollisionPair {
  int bodyA;
  int bodyB;
};

vector<BroadPhaseCollisionPair> getPotentialCollisionPairs(const Grid &grid) {
  vector<BroadPhaseCollisionPair> pairs;

  auto createPairKey = [](int id1, int id2) -> uint64_t {
    if (id1 > id2)
      swap(id1, id2);
    return (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);
  };

  unordered_set<uint64_t> addedPairs;

  for (int row = 0; row < grid.numRows; ++row) {
    for (int col = 0; col < grid.numCols; ++col) {
      const auto &currentCell = grid.cells[getCellIndex(grid, row, col)];

      for (size_t i = 0; i < currentCell.bodyIndex.size(); ++i) {
        for (size_t j = i + 1; j < currentCell.bodyIndex.size(); ++j) {
          int id1 = currentCell.bodyIndex[i];
          int id2 = currentCell.bodyIndex[j];
          uint64_t pairKey = createPairKey(id1, id2);

          if (addedPairs.insert(pairKey).second) {
            pairs.push_back({id1, id2});
          }
        }
      }

      const int dx[] = {1, 1, 0, -1};
      const int dy[] = {0, 1, 1, 1};

      for (int dir = 0; dir < 4; ++dir) {
        int newCol = col + dx[dir];
        int newRow = row + dy[dir];

        if (newCol >= 0 && newCol < grid.numCols && newRow >= 0 &&
            newRow < grid.numRows) {

          const auto &adjacentCell =
              grid.cells[getCellIndex(grid, newRow, newCol)];

          for (int id1 : currentCell.bodyIndex) {
            for (int id2 : adjacentCell.bodyIndex) {
              uint64_t pairKey = createPairKey(id1, id2);

              if (addedPairs.insert(pairKey).second) {
                pairs.push_back({id1, id2});
              }
            }
          }
        }
      }
    }
  }

  return pairs;
}

void updateBroadPhase(Grid &grid, const vector<Body> &bodies) {
  clearGrid(grid);
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].isActive) {
      insertBodyIntoGrid(grid, i, bodies[i]);
    }
  }
}

bool circleCircleCollision(const Circle &c1, const Circle &c2) {
  vec2 diff = vec2_sub(c1.center, c2.center);
  double distSq = vec2_length_squared(diff);
  double radiusSum = c1.radius + c2.radius;
  return distSq <= radiusSum * radiusSum;
}

bool aabbAabbCollision(const AABB &aabb1, const AABB &aabb2) {
  if (aabb1.max.x < aabb2.min.x || aabb1.min.x > aabb2.max.x)
    return false;
  if (aabb1.max.y < aabb2.min.y || aabb1.min.y > aabb2.max.y)
    return false;
  return true;
}

bool circleAabbCollision(const Circle &c, const AABB &aabb) {
  double closestX = max(aabb.min.x, min(c.center.x, aabb.max.x));
  double closestY = max(aabb.min.y, min(c.center.y, aabb.max.y));

  double distanceX = c.center.x - closestX;
  double distanceY = c.center.y - closestY;

  double distanceSquared = distanceX * distanceX + distanceY * distanceY;
  return distanceSquared <= (c.radius * c.radius);
}

bool checkCollision(const Body &bodyA, const Body &bodyB) {
  if (bodyA.shapeType == Body::SHAPE_CIRCLE &&
      bodyB.shapeType == Body::SHAPE_CIRCLE) {
    return circleCircleCollision(bodyA.shape.circle, bodyB.shape.circle);
  } else if (bodyA.shapeType == Body::SHAPE_AABB &&
             bodyB.shapeType == Body::SHAPE_AABB) {
    return aabbAabbCollision(bodyA.shape.aabb, bodyB.shape.aabb);
  } else if (bodyA.shapeType == Body::SHAPE_CIRCLE &&
             bodyB.shapeType == Body::SHAPE_AABB) {
    return circleAabbCollision(bodyA.shape.circle, bodyB.shape.aabb);
  } else if (bodyA.shapeType == Body::SHAPE_AABB &&
             bodyB.shapeType == Body::SHAPE_CIRCLE) {
    return circleAabbCollision(bodyB.shape.circle, bodyA.shape.aabb);
  }
  return false;
}

vector<BroadPhaseCollisionPair>
getActualCollisions(const vector<BroadPhaseCollisionPair> &potentialPairs,
                    const vector<Body> &bodies) {
  vector<BroadPhaseCollisionPair> actualCollisions;
  for (const auto &pair : potentialPairs) {
    const Body &bodyA = bodies[pair.bodyA];
    const Body &bodyB = bodies[pair.bodyB];
    if (checkCollision(bodyA, bodyB)) {
      actualCollisions.push_back(pair);
    }
  }
  return actualCollisions;
}

void resolveCircleCircleCollision(Body &a, Body &b) {
  vec2 distance = vec2_sub(b.position, a.position);
  vec2 normalVector = distance;
  vec2_normalize(normalVector);
  vec2 tangentVector = {-1.0 * normalVector.y, normalVector.x};

  vec2 relativeVelocity = vec2_sub(b.velocity, a.velocity);
  double velocitySeperation = vec2_dot(relativeVelocity, normalVector);

  double impulse =
      -1.0 * (1 + e) / (a.inverseMass + b.inverseMass) * velocitySeperation;
  a.velocity =
      vec2_sub(a.velocity, vec2_mul(normalVector, impulse * a.inverseMass));
  b.velocity =
      vec2_add(b.velocity, vec2_mul(normalVector, impulse * b.inverseMass));

  const double k = 0.4; // position correction factor (can be adjusted)
  double totalRadius = a.shape.circle.radius + b.shape.circle.radius;
  double penetrationDepth = totalRadius - vec2_length(distance);

  if (penetrationDepth > 0) {
    vec2 correction = vec2_mul(
        normalVector, (k * penetrationDepth) / (a.inverseMass + b.inverseMass));

    // Only move if body isn't static
    if (!a.isStatic) {
      a.position = vec2_sub(a.position, vec2_mul(correction, a.inverseMass));
    }
    if (!b.isStatic) {
      b.position = vec2_add(b.position, vec2_mul(correction, b.inverseMass));
    }
  }
}

void resolveAABBCollision(Body &a, Body &b) {
  // Calculate centers and half-sizes
  vec2 centerA = a.position;
  vec2 centerB = b.position;
  vec2 halfSizeA = vec2_mul(vec2_sub(a.shape.aabb.max, a.shape.aabb.min), 0.5);
  vec2 halfSizeB = vec2_mul(vec2_sub(b.shape.aabb.max, b.shape.aabb.min), 0.5);

  // Calculate delta between centers
  vec2 delta = vec2_sub(centerB, centerA);

  // Calculate overlap on each axis
  vec2 overlap = {halfSizeA.x + halfSizeB.x - fabs(delta.x),
                  halfSizeA.y + halfSizeB.y - fabs(delta.y)};

  // Determine collision normal based on smallest overlap
  vec2 normal;
  double penetration;
  if (overlap.x > 0 && overlap.y > 0) {
    if (overlap.x < overlap.y) {
      normal = (delta.x < 0) ? (vec2){-1.0, 0.0} : (vec2){1.0, 0.0};
      penetration = overlap.x;
    } else {
      normal = (delta.y < 0) ? (vec2){0.0, -1.0} : (vec2){0.0, 1.0};
      penetration = overlap.y;
    }

    // Calculate relative velocity
    vec2 relativeVel = vec2_sub(b.velocity, a.velocity);
    double normalVel = vec2_dot(relativeVel, normal);

    // Only resolve if objects are moving toward each other
    const double MIN_VELOCITY = 0.01;
    if (normalVel > -MIN_VELOCITY) {
      // Objects moving apart or too slow - just correct position
      if (!a.isStatic) {
        a.position = vec2_sub(a.position, vec2_mul(normal, penetration * 0.5));
      }
      if (!b.isStatic) {
        b.position = vec2_add(b.position, vec2_mul(normal, penetration * 0.5));
      }
      return;
    }

    // Calculate impulse
    const double RESTITUTION = 0.3;
    double totalInverseMass = a.inverseMass + b.inverseMass;
    if (totalInverseMass <= 0)
      return;

    // Apply restitution based on velocity
    double restitution = vec2_length(relativeVel) < 2.0 ? 0.0 : RESTITUTION;
    double j = -(1.0 + restitution) * normalVel / totalInverseMass;

    // Apply impulse
    vec2 impulse = vec2_mul(normal, j);
    if (!a.isStatic) {
      a.velocity = vec2_sub(a.velocity, vec2_mul(impulse, a.inverseMass));
    }
    if (!b.isStatic) {
      b.velocity = vec2_add(b.velocity, vec2_mul(impulse, b.inverseMass));
    }

    // Apply friction
    const double FRICTION = 0.4;
    vec2 tangent = {relativeVel.x - normalVel * normal.x,
                    relativeVel.y - normalVel * normal.y};
    double tangentLen = vec2_length(tangent);
    if (tangentLen > MIN_VELOCITY) {
      tangent = vec2_mul(tangent, 1.0 / tangentLen);
      double jt = -vec2_dot(relativeVel, tangent) / totalInverseMass;

      // Apply friction impulse
      vec2 frictionImpulse = vec2_mul(tangent, jt * FRICTION);
      if (!a.isStatic) {
        a.velocity =
            vec2_sub(a.velocity, vec2_mul(frictionImpulse, a.inverseMass));
      }
      if (!b.isStatic) {
        b.velocity =
            vec2_add(b.velocity, vec2_mul(frictionImpulse, b.inverseMass));
      }
    }

    // Position correction
    const double CORRECTION_PERCENT = 0.8;
    vec2 correction =
        vec2_mul(normal, (penetration / totalInverseMass) * CORRECTION_PERCENT);

    if (!a.isStatic) {
      a.position = vec2_sub(a.position, vec2_mul(correction, a.inverseMass));
    }
    if (!b.isStatic) {
      b.position = vec2_add(b.position, vec2_mul(correction, b.inverseMass));
    }

    // Update AABB positions
    if (!a.isStatic) {
      vec2 halfSize =
          vec2_mul(vec2_sub(a.shape.aabb.max, a.shape.aabb.min), 0.5);
      a.shape.aabb.min = vec2_sub(a.position, halfSize);
      a.shape.aabb.max = vec2_add(a.position, halfSize);
    }

    if (!b.isStatic) {
      vec2 halfSize =
          vec2_mul(vec2_sub(b.shape.aabb.max, b.shape.aabb.min), 0.5);
      b.shape.aabb.min = vec2_sub(b.position, halfSize);
      b.shape.aabb.max = vec2_add(b.position, halfSize);
    }
  }
}

void resolveCircleAABBCollision(Body &circleBody, Body &aabbBody,
                                bool circleIsA) {
  // Ensure we're working with the right shapes
  if (circleBody.shapeType != Body::SHAPE_CIRCLE ||
      aabbBody.shapeType != Body::SHAPE_AABB) {
    return;
  }

  // 1. Find closest point on AABB to circle center
  vec2 closestPoint = {
      max(aabbBody.shape.aabb.min.x,
          min(circleBody.position.x, aabbBody.shape.aabb.max.x)),
      max(aabbBody.shape.aabb.min.y,
          min(circleBody.position.y, aabbBody.shape.aabb.max.y))};

  // 2. Calculate distance vector from closest point to circle center
  vec2 distance = vec2_sub(circleBody.position, closestPoint);
  double distanceMag = vec2_length(distance);

  // Skip if no collision
  if (distanceMag > circleBody.shape.circle.radius) {
    return;
  }

  // 3. Calculate normal vector
  vec2 normal;
  const double EPSILON = 0.0001;
  if (distanceMag < EPSILON) {
    // Circle center is inside AABB, use vector from AABB center to circle
    // center
    vec2 aabbCenter = {
        (aabbBody.shape.aabb.min.x + aabbBody.shape.aabb.max.x) * 0.5,
        (aabbBody.shape.aabb.min.y + aabbBody.shape.aabb.max.y) * 0.5};
    distance = vec2_sub(circleBody.position, aabbCenter);
    distanceMag = vec2_length(distance);
    if (distanceMag < EPSILON) {
      // Objects are at same position, use arbitrary normal
      normal = {0.0, 1.0};
    } else {
      normal = vec2_mul(distance, 1.0 / distanceMag);
    }
  } else {
    normal = vec2_mul(distance, 1.0 / distanceMag);
  }

  // 4. Calculate penetration depth
  double penetrationDepth = circleBody.shape.circle.radius - distanceMag;

  // 5. Calculate relative velocity
  vec2 relativeVel = vec2_sub(circleBody.velocity, aabbBody.velocity);

  // 6. Calculate impact speed
  double impactSpeed = vec2_dot(relativeVel, normal);

  // Only resolve if objects are moving toward each other
  const double MIN_VELOCITY = 0.01;
  if (impactSpeed > -MIN_VELOCITY) {
    // Objects moving apart or too slow - just correct position
    const double POSITION_CORRECTION = 0.8;
    double totalInverseMass = circleBody.inverseMass + aabbBody.inverseMass;
    if (totalInverseMass > 0) {
      vec2 correction = vec2_mul(normal, (penetrationDepth / totalInverseMass) *
                                             POSITION_CORRECTION);
      if (!circleBody.isStatic) {
        circleBody.position = vec2_add(
            circleBody.position, vec2_mul(correction, circleBody.inverseMass));
      }
      if (!aabbBody.isStatic) {
        aabbBody.position = vec2_sub(
            aabbBody.position, vec2_mul(correction, aabbBody.inverseMass));
      }
    }
    return;
  }

  // 7. Calculate impulse
  double totalInverseMass = circleBody.inverseMass + aabbBody.inverseMass;
  if (totalInverseMass <= 0)
    return;

  // Apply restitution based on velocity
  double restitution = vec2_length(relativeVel) < 2.0 ? 0.0 : e;
  double j = -(1.0 + restitution) * impactSpeed / totalInverseMass;

  // 8. Apply impulse to velocities
  vec2 impulse = vec2_mul(normal, j);
  if (!circleBody.isStatic) {
    circleBody.velocity = vec2_add(circleBody.velocity,
                                   vec2_mul(impulse, circleBody.inverseMass));
  }
  if (!aabbBody.isStatic) {
    aabbBody.velocity =
        vec2_sub(aabbBody.velocity, vec2_mul(impulse, aabbBody.inverseMass));
  }

  // 9. Position correction
  const double POSITION_CORRECTION = 0.8;
  vec2 correction = vec2_mul(normal, (penetrationDepth / totalInverseMass) *
                                         POSITION_CORRECTION);
  if (!circleBody.isStatic) {
    circleBody.position = vec2_add(
        circleBody.position, vec2_mul(correction, circleBody.inverseMass));
  }
  if (!aabbBody.isStatic) {
    aabbBody.position =
        vec2_sub(aabbBody.position, vec2_mul(correction, aabbBody.inverseMass));
  }

  // 10. Apply friction
  const double FRICTION = 0.4;
  vec2 tangent = {-normal.y, normal.x}; // Perpendicular to normal
  double tangentSpeed = vec2_dot(relativeVel, tangent);

  // Calculate and clamp friction impulse
  double frictionImpulse = -tangentSpeed / totalInverseMass;
  double maxFriction = FRICTION * fabs(j);
  frictionImpulse = max(-maxFriction, min(frictionImpulse, maxFriction));

  vec2 frictionVector = vec2_mul(tangent, frictionImpulse);
  if (!circleBody.isStatic) {
    circleBody.velocity = vec2_add(
        circleBody.velocity, vec2_mul(frictionVector, circleBody.inverseMass));
  }
  if (!aabbBody.isStatic) {
    aabbBody.velocity = vec2_sub(
        aabbBody.velocity, vec2_mul(frictionVector, aabbBody.inverseMass));
  }

  // Update circle position
  if (!circleBody.isStatic) {
    circleBody.shape.circle.center = circleBody.position;
  }

  // Update AABB position
  if (!aabbBody.isStatic) {
    vec2 halfSize = vec2_mul(
        vec2_sub(aabbBody.shape.aabb.max, aabbBody.shape.aabb.min), 0.5);
    aabbBody.shape.aabb.min = vec2_sub(aabbBody.position, halfSize);
    aabbBody.shape.aabb.max = vec2_add(aabbBody.position, halfSize);
  }
}

void resolveCollisions(vector<BroadPhaseCollisionPair> &collisions,
                       vector<Body> &bodies) {
  for (const auto &pair : collisions) {
    Body &bodyA = bodies[pair.bodyA];
    Body &bodyB = bodies[pair.bodyB];

    if (bodyA.shapeType == Body::SHAPE_CIRCLE &&
        bodyB.shapeType == Body::SHAPE_CIRCLE) {
      resolveCircleCircleCollision(bodyA, bodyB);
    } else if (bodyA.shapeType == Body::SHAPE_AABB &&
               bodyB.shapeType == Body::SHAPE_AABB) {
      resolveAABBCollision(bodyA, bodyB);
    } else {
      if (bodyA.shapeType == Body::SHAPE_CIRCLE) {
        resolveCircleAABBCollision(bodyA, bodyB, true);
      } else {
        resolveCircleAABBCollision(bodyB, bodyA, false);
      }
    }
  }
}
