#include "planar_quadrotor_visualizer.h"

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

    float max_image_x = 10.0;
    
    float image_scale = SCREEN_WIDTH/ max_image_x/ 2;


    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

   int image_x = static_cast<int>(SCREEN_WIDTH / 2  *q_x/image_scale + SCREEN_WIDTH/2);
   int image_y = static_cast<int>(SCREEN_HEIGHT / 2 * q_y/image_scale + SCREEN_HEIGHT/2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    filledCircleColor(gRenderer.get(), image_x, image_y, 30, 0xFF0000FF);
}