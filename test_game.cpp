// game.cpp
#include "engine.cpp"
#include <SDL2/SDL.h>
#include <iostream>
#include <vector>

// Window settings
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float PIXELS_PER_METER = 20.0f; // Scale factor for conversion
const float METER_TO_PIXEL = PIXELS_PER_METER;
const float PIXEL_TO_METER = 1.0f / PIXELS_PER_METER;

// Convert physics coordinates to screen coordinates
SDL_Point worldToScreen(const vec2 &position) {
  return {static_cast<int>(position.x * METER_TO_PIXEL + WINDOW_WIDTH / 2),
          static_cast<int>(WINDOW_HEIGHT -
                           (position.y * METER_TO_PIXEL + WINDOW_HEIGHT / 2))};
}

void renderCircle(SDL_Renderer *renderer, const Body &body) {
  if (body.shapeType != Body::SHAPE_CIRCLE)
    return;

  SDL_Point center = worldToScreen(body.position);
  int radius = static_cast<int>(body.shape.circle.radius * METER_TO_PIXEL);

  // Draw circle using SDL's primitive drawing
  for (int w = 0; w < radius * 2; w++) {
    for (int h = 0; h < radius * 2; h++) {
      int dx = radius - w;
      int dy = radius - h;
      if ((dx * dx + dy * dy) <= (radius * radius)) {
        SDL_RenderDrawPoint(renderer, center.x + dx, center.y + dy);
      }
    }
  }
}

void renderAABB(SDL_Renderer *renderer, const Body &body) {
  if (body.shapeType != Body::SHAPE_AABB)
    return;

  SDL_Point min = worldToScreen({body.shape.aabb.min.x, body.shape.aabb.min.y});
  SDL_Point max = worldToScreen({body.shape.aabb.max.x, body.shape.aabb.max.y});

  SDL_Rect rect = {min.x, max.y, max.x - min.x, min.y - max.y};

  SDL_RenderDrawRect(renderer, &rect);
}

vec2 screenToWorld(int screenX, int screenY) {
  return vec2{(screenX - WINDOW_WIDTH / 2) * PIXEL_TO_METER,
              (WINDOW_HEIGHT / 2 - screenY) * PIXEL_TO_METER};
}

int main() {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "SDL initialization failed: " << SDL_GetError() << std::endl;
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow(
      "Physics Engine Test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
      WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);

  if (!window) {
    std::cout << "Window creation failed: " << SDL_GetError() << std::endl;
    return 1;
  }

  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  if (!renderer) {
    std::cout << "Renderer creation failed: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Initialize physics world
  World world;
  initWorld(world);

  addCircle(world, {0, 10}, 1.0, 1.0);
  addBox(world, {5, 15}, {2, 2}, 2.0);
  addCircle(world, {-3, 20}, 0.8, 0.5);

  // Main loop
  bool running = true;
  SDL_Event event;
  Uint32 lastTime = SDL_GetTicks();
  const double fixedTimeStep = 1.0 / 60.0;

  while (running) {
    // Handle events
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        vec2 worldPos = screenToWorld(mouseX, mouseY);

        if (event.button.button == SDL_BUTTON_LEFT) {
          // Left click - add box
          addBox(world, worldPos, vec2{2, 2}, 1.0); // 2x2 meter box with mass 1
        } else if (event.button.button == SDL_BUTTON_RIGHT) {
          // Right click - add circle
          addCircle(world, worldPos, 1.0,
                    1.0); // 1 meter radius circle with mass 1
        }
      }
    }

    // Rest of your main loop (physics update and rendering) remains the same
    Uint32 currentTime = SDL_GetTicks();
    double deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    stepWorld(world, fixedTimeStep);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (const auto &body : world.bodies) {
      if (body.shapeType == Body::SHAPE_CIRCLE) {
        renderCircle(renderer, body);
      } else {
        renderAABB(renderer, body);
      }
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(16);
  }

  // Cleanup
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
