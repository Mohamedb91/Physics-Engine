#include <stdio.h>
#include <math.h>
#include "inc/SDL.h"
#include "inc/Circle.h"

#undef main

// Constants
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1150
#define GRAVITY 0.6f
#define DELTA_TIME 0.5f
#define KINETIC_ENERGY_LOSS_X 1    //0.7f
#define MIN_VELOCITY_THRESHOLD 1  //1.75f
#define FRICTION 1     //0.9f
#define GROUND_FRICTION 1 //0.825f  // Stronger friction when rolling on the ground
#define MAX_CIRCLES 130
float KinecticEnergyLossy = 1; //0.9;

// Circle data structure
typedef struct {
    Circle* circle;
    float vy;
    float ay;
    float vx;
    float ax;
    float totalV;
} CircleData;

CircleData circles[MAX_CIRCLES];
int num_circles = 0;

void update_physics(Circle* circle, float* vy, float* ay, float* vx, float* ax, SDL_Rect* box, float* totalV) {
    *totalV = sqrt((*vx) * (*vx) + (*vy) * (*vy));

    *vy += (*ay * DELTA_TIME);
    circle->y += (*vy * DELTA_TIME);

    *vx += (*ax * DELTA_TIME);
    circle->x += (*vx * DELTA_TIME);

    // Apply friction
    if (circle->y + circle->radius >= (box->y + box->h)) {
        *vx *= GROUND_FRICTION;  // Apply stronger friction when on the ground
    } else {
        *vx *= FRICTION;
    }

    // Right wall collision
    if (circle->x + circle->radius >= (box->x + box->w)) {
        circle->x = (box->x + box->w) - circle->radius;
        *vx = -*vx;
        *ax = -*ax * KINETIC_ENERGY_LOSS_X;
    }

    // Left wall collision
    if (circle->x - circle->radius <= box->x) {
        circle->x = box->x + circle->radius;
        *vx = -*vx;
        *ax = -*ax * KINETIC_ENERGY_LOSS_X;
    }

    // Bottom wall collision
    if (circle->y + circle->radius >= (box->y + box->h)) {
        circle->y = (box->y + box->h) - circle->radius;
        *vy = -*vy * KinecticEnergyLossy;
        *ay = -*ay;

        // if (KinecticEnergyLossy > 0) {
        //     KinecticEnergyLossy -= 0.025;
        // }

        // Stop bouncing if velocity is below the threshold
        // if (fabs(*vy) < MIN_VELOCITY_THRESHOLD) {
        //     *vy = 0;
        // }
    }

    // Top wall collision
    if (circle->y - circle->radius <= box->y) {
        circle->y = box->y + circle->radius;
        *vy = -*vy;
        *ay = -*ay;
    }
}

void handle_collision_2Circles(Circle* circle, float* vx1, float* vy1, float* ax1, float* ay1, float* totalV1, 
                               Circle* circle2, float* vx2, float* vy2, float* ax2, float* ay2, float* totalV2) {
    float dx = circle2->x - circle->x;
    float dy = circle2->y - circle->y;
    float distance = sqrt(dx * dx + dy * dy);

    if (distance <= (circle->radius + circle2->radius)) {

        // https://www.vobarian.com/collisions/2dcollisions2.pdf
        // Find the unit normal and unit tangent vectors
        float un_x = dx / distance;
        float un_y = dy / distance;

        float ut_x = -un_y;
        float ut_y = un_x;

        // Create the initial velocity vectors
        // Resolve the velocity vectors into normal and tangential components
        float v1n = un_x * *vx1 + un_y * *vy1;
        float v1t = ut_x * *vx1 + ut_y * *vy1;
        float v2n = un_x * *vx2 + un_y * *vy2;
        float v2t = ut_x * *vx2 + ut_y * *vy2;

        // The tangential components of the velocity do not change
        float v1t_prime = v1t;
        float v2t_prime = v2t;

        // The normal components of the velocity are calculated using 1D collision formulas
        float v1n_prime = (v1n * (circle->mass - circle2->mass) + 2 * circle2->mass * v2n) / (circle->mass + circle2->mass);
        float v2n_prime = (v2n * (circle2->mass - circle->mass) + 2 * circle->mass * v1n) / (circle->mass + circle2->mass);

        // Convert the scalar normal and tangential velocities back to vectors
        float v1n_prime_x = v1n_prime * un_x;
        float v1n_prime_y = v1n_prime * un_y;
        float v1t_prime_x = v1t_prime * ut_x;
        float v1t_prime_y = v1t_prime * ut_y;

        float v2n_prime_x = v2n_prime * un_x;
        float v2n_prime_y = v2n_prime * un_y;
        float v2t_prime_x = v2t_prime * ut_x;
        float v2t_prime_y = v2t_prime * ut_y;

        // 7. Find the final velocity vectors by adding the normal and tangential components
        *vx1 = v1n_prime_x + v1t_prime_x;
        *vy1 = v1n_prime_y + v1t_prime_y;
        *vx2 = v2n_prime_x + v2t_prime_x;
        *vy2 = v2n_prime_y + v2t_prime_y;

        // Adjust positions to prevent sticking
        float overlap = (circle->radius + circle2->radius) - distance;
        float normX = dx / distance;
        float normY = dy / distance;

        circle->x -= normX * overlap / 2;
        circle->y -= normY * overlap / 2;
        circle2->x += normX * overlap / 2;
        circle2->y += normY * overlap / 2;

        // Swap accelerations
        float temp_ax = *ax1;
        float temp_ay = *ay1;
        *ax1 = *ax2;
        *ay1 = *ay2;
        *ax2 = temp_ax;
        *ay2 = temp_ay;
    }
}

void add_circle(float x, float y, float radius, SDL_Color color, float mass) {
    if (num_circles >= MAX_CIRCLES) return;
    circles[num_circles].circle = create_circle(x, y, radius, color, mass);
    circles[num_circles].vy = (rand() % 10);
    circles[num_circles].ay = 0;  //GRAVITY * mass / mass
    circles[num_circles].vx = (rand() % 10);
    circles[num_circles].ax = 0;  //10 * mass / mass;
    circles[num_circles].totalV = sqrt(circles[num_circles].vx * circles[num_circles].vx + circles[num_circles].vy * circles[num_circles].vy);
    num_circles++;
}

void add_multiple_circles(int count, int x_min, int x_max, int y_min, int y_max, float radius, SDL_Color color, float mass) {
    for (int i = 0; i < count; i++) {
        add_circle((rand() % (x_max - x_min + 1)) + x_min, 
                   (rand() % (y_max - y_min + 1)) + y_min, 
                   radius, 
                   color, 
                   mass);
    }
}

int main() {
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;

    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    SDL_Rect box;
    box.w = WINDOW_WIDTH - 20;
    box.h = WINDOW_HEIGHT - 15;
    box.x = 10;
    box.y = 7.5;
    SDL_RenderDrawRect(renderer, &box);


    // Frequently used colours
    SDL_Color red = {255, 0, 0, 255};
    SDL_Color green = {0, 255, 0, 255};
    SDL_Color blue = {0, 0, 255, 255};
    SDL_Color yellow = {255, 255, 0, 255};

    // number of circles of each colour
    int red_count = 35;
    int blue_count = 35;
    int green_count = 30;
    int yellow_count = 30;

    // Adding circles of different colour 
    add_multiple_circles(red_count, 0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, 25, red, 5);
    add_multiple_circles(blue_count, 0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, 25, blue, 5);
    add_multiple_circles(green_count, 0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, 25, green, 5);
    add_multiple_circles(yellow_count, 0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, 25, yellow, 5);

    // Cursor position (TEMP)
    int x, y;

    int quit = 0;
    SDL_Event event;

    while (!quit) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderDrawRect(renderer, &box);
       
        SDL_GetMouseState(&x, &y);
        printf("Mouse Position: (%d, %d)\n", x, y);
        fflush(stdout); // Flush the buffer to ensure immediate output

        for (int i = 0; i < num_circles; i++) {
            update_physics(circles[i].circle, &circles[i].vy, &circles[i].ay, &circles[i].vx, &circles[i].ax, &box, &circles[i].totalV);
        }

        for (int i = 0; i < num_circles; i++) {
            for (int j = i + 1; j < num_circles; j++) {
                handle_collision_2Circles(circles[i].circle, &circles[i].vx, &circles[i].vy, &circles[i].ax, &circles[i].ay, &circles[i].totalV,
                                          circles[j].circle, &circles[j].vx, &circles[j].vy, &circles[j].ax, &circles[j].ay, &circles[j].totalV);
            }
        }

        for (int i = 0; i < num_circles; i++) {
            draw_circle(renderer, circles[i].circle);
        }

        SDL_RenderPresent(renderer);

        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    quit = 1;
                    break;
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_ESCAPE)
                        quit = 1;
                    break;
                default:
                    break;
            }
        }
        SDL_Delay(20); // Slow down the main loop for better visual effect
    }

    for (int i = 0; i < num_circles; i++) {
        destroy_circle(circles[i].circle);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
