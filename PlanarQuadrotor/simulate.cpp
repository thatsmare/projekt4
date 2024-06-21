/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include <matplot/matplot.h>
#include "simulate.h"
#include <cmath>

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {          //MACIERZ LQR
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    /*Q.diagonal() << 0.004, 0.004, 400, 0.004, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;*/

    Q.diagonal() << 50, 50, 50, 5, 50, 2.5 / 2 / M_PI;         //LQR TESTING
    R.row(0) << 0.005, 0.0025;
    R.row(1) << 0.0025, 0.005;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void generate_sound(Uint8* stream, int len)
{
    const int amplitude = 1;  //amplituda dźwięku
    const double sampling_frequency = 44100.0;
    const int sine_wave_frequency = 440; // 440 Hz, standard A4 note
    double phase = 0.0;
    double phase_increment = 2.0 * M_PI * sine_wave_frequency / sampling_frequency;
    int16_t* buffer = (int16_t*)stream;
    int length = len / 2; // len is in bytes, we want number of samples

        for (int i = 0; i < length; ++i) 
        {
            buffer[i] = (int16_t)(amplitude * sin(phase));
            phase += phase_increment;
            if (phase >= 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    Uint32 start_time = 0;
    start_time = SDL_GetTicks();

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]  <------ DONE
     * 2. Update PlanarQuadrotor from simulation when goal is changed   <------ DONE
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0; //begin
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor <------ DONE
     * 2. Plot trajectory using matplot++ when key 'p' is clicked  <------DONE
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0)
        {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            return -1;
        }

        //audio specification config

        SDL_AudioSpec spec;
        spec.freq = 44100.0;           //czestotliwosc probkowania
        spec.format = AUDIO_U8;
        spec.channels = 2;              //stereo - jak leci w lewo to lewy głosnik glosniej itp
        spec.samples = 4096;
        spec.callback = NULL;

        SDL_AudioSpec obtained_spec;
        SDL_AudioDeviceID audio_device = SDL_OpenAudioDevice(NULL, 0, &spec, NULL, 0);
        if (audio_device == 0) {
            std::cout << "SDL Audio Error: " << SDL_GetError() << std::endl;
            SDL_Quit();
            return -1;
        }

        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float x0, y0;           //new coordinates
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            //update of history - as frequent as possible to get good accuracy
            Eigen::VectorXf state = quadrotor.GetState();
            x_history.push_back(state(0) * 1280 + 640);
            y_history.push_back(state(1) * 760 + 360);
            theta_history.push_back(state(2));
            Uint32 current_time = SDL_GetTicks();
            float time = static_cast<float>(current_time - start_time) / 1000.0f;
            time_history.push_back(time);

            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    if (e.button.button == SDL_BUTTON_LEFT)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                        x0 = x;
                        y0 = y;
                        x0 = (x0 - 640) / 1280;
                        y0 = (y0 - 360) / 760;
                        goal_state << x0, y0, 0, 0, 0, 0;
                        quadrotor.SetGoal(goal_state);
                    }
                }
                else if (e.type == SDL_KEYDOWN) {
                    if (e.key.keysym.sym == SDLK_p) {
                        using namespace matplot;

                        auto fig = figure(true);
                        fig->size(1600, 900);

                        subplot(3, 1, 0);
                        plot(time_history, x_history);
                        title("x history");
                        ylabel("X position");

                        subplot(3, 1, 1);
                        plot(time_history, y_history);
                        title("y history");
                        ylabel("Y position");

                        subplot(3, 1, 2);
                        plot(time_history, theta_history);
                        title("theta history");
                        ylabel("theta");

                        show();
                    }
                }
            }

            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    //SDL_CloseAudioDevice(audio_device);
    SDL_Quit();

    return 0;
}


int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
