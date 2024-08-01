#ifndef CIRCLE_H
#define CIRCLE_H

#include "SDL.h"

typedef struct {
    int x;
    int y;
    int radius;
    SDL_Color colour;
    float mass;
} Circle;

Circle* create_circle(int x, int y, int radius, SDL_Color colour, float mass);
void destroy_circle(Circle* circle);
void draw_circle(SDL_Renderer* renderer, Circle* circle);


#endif