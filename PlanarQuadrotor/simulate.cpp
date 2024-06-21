/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include <matplot/matplot.h>
#include "simulate.h"
#include <cmath>

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
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

    Q.diagonal() << 50, 50, 50, 5, 50, 2.5 / 2 / M_PI;
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

//---------------------------------------AUDIO----------------------------------------------------------------
void generate_sound(int16_t* buffer, int length) {
    const int amplitude = 1000; // Amplituda dźwięku
    const double sampleRate = 44100.0; // Częstotliwość próbkowania
    const double frequency = 220.0; // Częstotliwość A3
    length = length / 2; // Długość bufora w próbkach (dla 16-bitowego dźwięku)

    for (int i = 0; i < length; ++i) {
        double time = i / sampleRate;
        double value = amplitude * sin(2.0 * M_PI * frequency * time);
        buffer[i] = static_cast<int16_t>(value);
    }
}
//---------------------------------------------------------------------------------------------

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
     *
     TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor <------ DONE
     * 2. Plot trajectory using matplot++ when key 'p' is clicked  <------DONE
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0; //begin
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_AudioSpec spec;
    spec.freq = 44100; // Częstotliwość próbkowania
    spec.format = AUDIO_S16SYS; // Format dźwięku
    spec.channels = 1; // Liczba kanałów
    spec.samples = 32768; // Rozmiar bufora próbek
    spec.callback = NULL; 
    spec.userdata = NULL;

    SDL_AudioDeviceID deviceId = SDL_OpenAudioDevice(NULL, 0, &spec, NULL, 0);
    if (deviceId == 0) {
        std::cerr << "SDL audio device initialization failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    int16_t* audioBuffer = new int16_t[spec.samples];

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        bool at_goal;
        int x, y;
        float x0, y0;           //new coordinates
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf current_state = quadrotor.GetState();
        SDL_PauseAudioDevice(deviceId, 0);

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

            current_state = quadrotor.GetState();

            at_goal = (current_state - goal_state).norm() < 0.02; // Tolerancja odległości dla uznania, że osiągnięto cel

            if (!at_goal)
            {
                generate_sound(audioBuffer, spec.samples * sizeof(int16_t)); // Generowanie dźwięku A3
                SDL_QueueAudio(deviceId, audioBuffer, spec.samples * sizeof(int16_t));
                SDL_PauseAudioDevice(deviceId, 0);
            }
            else {
                SDL_PauseAudioDevice(deviceId, 1); // Wstrzymanie odtwarzania dźwięku jeśli osiągnięto cel
            }
            current_state.setZero(); // Czyszczenie current_state
        }
    }
    SDL_CloseAudioDevice(deviceId);
    delete[] audioBuffer;
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
