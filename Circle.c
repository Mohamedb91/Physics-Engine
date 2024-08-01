#include "inc/Circle.h"
#include <stdlib.h>
#include <math.h>

Circle* create_circle(int x, int y, int radius, SDL_Color colour, float mass) {

    Circle* circle = (Circle*)malloc(sizeof(Circle));
    if (circle != NULL) {
        circle->x = x;
        circle->y = y;
        circle->radius = radius;
        circle->colour = colour; 
        circle->mass = mass;
    }
    return circle;
}

void destroy_circle(Circle* circle) {

    if (circle != NULL) {
        free(circle);
    }
}

void draw_circle(SDL_Renderer* renderer, Circle* circle) {

    int x0 = circle->x;
    int y0 = circle->y;
    int radius = circle-> radius;
    SDL_SetRenderDrawColor(renderer, circle->colour.r, circle->colour.g, circle->colour.b, circle->colour.a);

    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        SDL_RenderDrawLine(renderer, x0 - x, y0 + y, x0 + x, y0 + y);
        SDL_RenderDrawLine(renderer, x0 - y, y0 + x, x0 + y, y0 + x);
        SDL_RenderDrawLine(renderer, x0 - x, y0 - y, x0 + x, y0 - y);
        SDL_RenderDrawLine(renderer, x0 - y, y0 - x, x0 + y, y0 - x);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }

        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }

}
