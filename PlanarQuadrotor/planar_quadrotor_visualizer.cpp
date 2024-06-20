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
    q_x = state[0] * 1280 + 640;
    q_y = state[1] * 760 + 360;
    q_theta = state[2];

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);

    float half_length = 50;
    float half_height = 10;
    int half_width_screen = SCREEN_WIDTH/2;
    int half_hight_screen = SCREEN_HEIGHT/2;
    float proppeler_radius = 10;

    //https://math.stackexchange.com/questions/2581058/rotating-rectangle-by-its-center#:~:text=1%20Answer,-Sorted%20by%3A&text=When%20you%20rotate%20the%20point,θ%2Bycosθ).&text=If%20the%20center%20of%20the,%2C%20rotate%2C%20then%20translate%20back.
    // x_output = x_original*cos(theta) - y_original*sin(theta)
    // y_output = x_original*sin(theta) + y_original*cos(theta)

    //main body of the drone 
    float x_1 = q_x + half_length * cos(q_theta) - half_height * sin(q_theta);
    float x_2 = q_x - half_length * cos(q_theta) - half_height * sin(q_theta);
    float x_3 = q_x - half_length * cos(q_theta) + half_height * sin(q_theta);
    float x_4 = q_x + half_length * cos(q_theta) + half_height * sin(q_theta);

    float y_1 = q_y + half_length * sin(q_theta) + half_height * cos(q_theta);
    float y_2 = q_y - half_length * sin(q_theta) + half_height * cos(q_theta);
    float y_3 = q_y - half_length * sin(q_theta) - half_height * cos(q_theta);
    float y_4 = q_y + half_length * sin(q_theta) - half_height * cos(q_theta);

    const Sint16 x_base[] = {x_1, x_2, x_3, x_4};
    const Sint16 y_base[] = {y_1, y_2, y_3, y_4}; 

    filledPolygonColor(gRenderer.get(), x_base, y_base, 4, 0xFFFF0000);

    //proppelers
    float p_x1 = q_x + (half_length*3/4-proppeler_radius)*cos(q_theta) -half_height*3 *sin(q_theta);
    float p_y1 = q_y + (half_length*3/4-proppeler_radius) * sin(q_theta) - half_height*3 * cos(q_theta);
    filledCircleColor(gRenderer.get(), p_x1, p_y1,proppeler_radius,0xFFFF0000 );

    float p_x2 = q_x + (half_length*3/4+proppeler_radius)*cos(q_theta) -half_height* 3 *sin(q_theta);
    float p_y2 = q_y + (half_length*3/4+proppeler_radius) * sin(q_theta) - half_height*3 * cos(q_theta);
    filledCircleColor(gRenderer.get(), p_x2, p_y2,proppeler_radius,0xFFFF0000 );
    
    float p_x3 = q_x - (half_length*3/4+proppeler_radius)*cos(q_theta) -half_height* 3 *sin(q_theta);
    float p_y3 = q_y - (half_length*3/4+proppeler_radius) * sin(q_theta) - half_height*3 * cos(q_theta);
    filledCircleColor(gRenderer.get(), p_x3, p_y3,proppeler_radius,0xFFFF0000 );
    
    float p_x4 = q_x - (half_length*3/4-proppeler_radius)*cos(q_theta) -half_height* 3 *sin(q_theta);
    float p_y4 = q_y - (half_length*3/4-proppeler_radius) * sin(q_theta) - half_height*3 * cos(q_theta);
    filledCircleColor(gRenderer.get(), p_x4, p_y4,proppeler_radius,0xFFFF0000 );

    //proppelers' base
    float pb_x1 = q_x + half_length*3/4*cos(q_theta) - half_height*3*sin(q_theta);
    float pb_y1 = q_y + half_length*3/4*sin(q_theta) - half_height*3*cos(q_theta);

    float pb_x2 = q_x + half_length*3/4*cos(q_theta) - half_height*sin(q_theta);
    float pb_y2 = q_y + half_length*3/4*sin(q_theta) - half_height*cos(q_theta);
    const Sint16 px_base_left[] = {pb_x1,pb_x2};
    const Sint16 py_base_left[] = {pb_y1,pb_y2};
    filledPolygonColor(gRenderer.get(), x_base, y_base, 4, 0xFFFF0000);


}

     //std::cout << q_x  << " , " << q_y << std::endl;
