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

     //std::cout << q_x  << " , " << q_y << std::endl;

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
   filledCircleColor(gRenderer.get(), q_x + SCREEN_WIDTH/2, q_y + SCREEN_HEIGHT/2, 30, 0xFF0000FF);         //added 640, 360
  /* int width = 100;
   int height = 10;
    SDL_Rect rectangle = { q_x + SCREEN_WIDTH/2 - width/2 , q_y + SCREEN_HEIGHT/2 - height/2, width, height };  // x, y, width, height
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0xFF, 0x00);  // pink apparently
    SDL_RenderFillRect(gRenderer.get(), &rectangle);
  */


}