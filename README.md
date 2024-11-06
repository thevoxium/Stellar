
# Stellar Engine

A lightweight 2D physics engine written in C++. Stellar provides a simple and efficient foundation for physics-based games and simulations.


## Quick Start

### Prerequisites
- C++ compiler with C++11 support
- SDL2 library

### Installation

#### macOS (using Homebrew)
```bash
brew install sdl2
```

#### Linux (Debian/Ubuntu)
```bash
sudo apt-get install libsdl2-dev
```

### Building

```bash
# Build with SDL2 visualization
g++ test_game.cpp -std=c++11 -o game -I/opt/homebrew/include -L/opt/homebrew/lib -lSDL2
```

### Running the Demo
```bash
./game
```

## Usage Example

```cpp

int main() {
  World world;
  initWorld(world);

  int circleId = addCircle(world, {0, 10}, 1.0, 1.0);
  int boxId = addBox(world, {5, 10}, {2, 2}, 2.0);

  Body &circle = world.bodies[circleId];
  std::cout << "Circle area: " << calculateArea(circle) << "\n";
  std::cout << "Circle MOI: " << calculateMOI(circle) << "\n";

  double fixedTimeStep = 1.0 / 60.0; // 60 Hz simulation
  for (int i = 0; i < 100; i++) {
    stepWorld(world, fixedTimeStep);

    std::cout << "Step " << i << ":\n";
    std::cout << "Circle position: " << world.bodies[circleId].position.x
              << ", " << world.bodies[circleId].position.y << "\n";
    std::cout << "Box position: " << world.bodies[boxId].position.x << ", "
              << world.bodies[boxId].position.y << "\n\n";
  }

  return 0;
}

```

ToDo

## Phase 1: Core Mathematics and Basic Motion
### Vector Operations

Vector Implementations are Done!

- ~~Vector2D struct definition (x, y components)~~
- ~~Vector operation functions:~~
  - ~~vec2_add, vec2_sub, vec2_mul~~
  - ~~vec2_dot, vec2_cross~~
  - ~~vec2_normalize~~
  - ~~vec2_rotate~~
  - ~~vec2_length, vec2_length_squared~~

Basic Motion Implementation is Done!

### Basic Motion
- ~~Body struct (position, velocity, acceleration)~~
- ~~Force functions:~~
  - ~~apply_force~~
  - ~~apply_impulse~~
- ~~Integration functions:~~
  - ~~integrate_linear_motion~~
  - ~~apply_gravity~~
- ~~World struct to hold all bodies~~

## Phase 2: Shapes and Collision Detection
### Basic Shapes
- ~~Circle struct (position, radius)~~
- ~~AABB struct (min, max points)~~
- Polygon struct (vertex array, vertex count)
- ~~Shape union struct with type enum~~
- ~~Point containment test functions~~

### Collision Detection
- Broad phase functions:
  - grid_system struct and functions
  - quadtree struct and functions
- Collision pair struct
- Narrow phase functions:
  - check_circle_circle
  - check_aabb_aabb
  - check_circle_aabb
  - check_polygon_polygon (SAT)

## Phase 3: Collision Response
### Basic Resolution
- Contact struct (normal, depth, points)
- Resolution functions:
  - resolve_collision_positions
  - resolve_collision_velocities
  - apply_momentum_conservation
  - resolve_penetration

### Material Properties
- Material struct:
  - restitution
  - friction_static
  - friction_dynamic
  - density
- Material resolution functions

## Phase 4: Constraints
### Basic Constraints
- Constraint struct with type enum
- Constraint functions:
  - solve_distance_constraint
  - solve_point_constraint
  - solve_rotation_constraint
  - solve_angle_limits

### Advanced Joints
- Spring struct and functions
- Rope struct and functions
- Joint struct variations
- Joint solver functions

## Phase 5: Advanced Features
### Forces and Effects
- Force generator struct
- Force application functions:
  - apply_drag
  - apply_buoyancy
  - apply_wind
- Raycast struct and functions

### Performance Optimization
- Body state flags (active/sleeping)
- Island struct for connected bodies
- Continuous collision functions
- Spatial partitioning optimization

## Phase 6: Special Features
### Soft Body Physics
- Spring-mass struct
- Soft body solver functions
- Cloth struct and functions
- Rope simulation functions

### Particles
- Particle struct
- Particle system functions
- Particle emitter struct
- Fluid particle functions

