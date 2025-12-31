#include "rendering/Ground.h"
#include "rendering/Camera.h"

Ground::Ground(int spacing)
    : grid_spacing(spacing), line_r(60), line_g(60), line_b(60) {}

void Ground::draw(SDL_Renderer* renderer, const Camera* camera,
                  int screenWidth, int screenHeight) {
    double left = camera->camera_x - screenWidth / 2;
    double right = camera->camera_x + screenWidth / 2;
    double top = camera->camera_y - screenHeight / 2;
    double bottom = camera->camera_y + screenHeight / 2;

    int start_x = static_cast<int>(left / grid_spacing) * grid_spacing;
    int start_y = static_cast<int>(top / grid_spacing) * grid_spacing;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, line_r, line_g, line_b, 120);

    for (int x = start_x; x < right; x += grid_spacing) {
        int screen_x = camera->worldToScreenX(x, screenWidth);
        SDL_RenderDrawLine(renderer, screen_x, 0, screen_x, screenHeight);
    }

    for (int y = start_y; y < bottom; y += grid_spacing) {
        int screen_y = camera->worldToScreenY(y, screenHeight);
        SDL_RenderDrawLine(renderer, 0, screen_y, screenWidth, screen_y);
    }
}
