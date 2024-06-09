#include "planar_quadrotor_visualizer.h"
#include <iostream>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;


    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);

    float half_lenght = 30;
    float half_height = 10;
    int half_width_screen = SCREEN_WIDTH/2;
    int half_hight_screen = SCREEN_HEIGHT/2;

    // x_output = x_original*cos(theta) - y_original*sin(theta)
    // y_output = x_original*sin(theta) + y_original*cos(theta)
    float x_1 = q_x + half_width_screen - half_lenght;
    float x_2 = q_x + half_width_screen + half_lenght;
    float x_3 = q_x + half_width_screen - half_lenght;
    float x_4 = q_x + half_width_screen + half_lenght;

    float y_1 = q_y + half_hight_screen - half_height;
    float y_2 = q_y + half_hight_screen - half_height;
    float y_3 = q_y + half_hight_screen + half_height;
    float y_4 = q_y + half_hight_screen + half_height;


    SDL_RenderDrawLine(gRenderer.get(), x_1, y_1, x_2, y_2);
    SDL_RenderDrawLine(gRenderer.get(), x_1, y_1, x_3, y_3);
    SDL_RenderDrawLine(gRenderer.get(), x_2, y_2, x_4, y_4);
    SDL_RenderDrawLine(gRenderer.get(), x_3, y_3, x_4, y_4);



}

     //std::cout << q_x  << " , " << q_y << std::endl;

 /*   filledCircleColor(gRenderer.get(), q_x + SCREEN_WIDTH/2, q_y + SCREEN_HEIGHT/2, 20, 0xFFFFFF00);         //added 640, 360
    int width = 100;
    int height = 10;
    SDL_Rect rectangle = { q_x + SCREEN_WIDTH/2 - width/2 , q_y + SCREEN_HEIGHT/2 - height/2, width, height };  // x, y, width, height
    SDL_RenderFillRect(gRenderer.get(), &rectangle);
    */