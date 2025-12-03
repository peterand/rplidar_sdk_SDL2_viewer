#include <SDL2/SDL.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <cmath>
#include <cstdio>
#include <cstddef>

using namespace sl;

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define MAX_DISTANCE_MM 8000.0   // Adjust to your lidar's max range

int main(int argc, const char * argv[])
{
    // -----------------------------
    // Create communication channel
    // -----------------------------
    const char * device = "/dev/ttyUSB0";
    int baudrate = 115200;

    auto channelResult = createSerialPortChannel(device, baudrate);
    if (!channelResult) {
        std::fprintf(stderr, "Error: cannot create serial channel to %s\n", device);
        return -1;
    }

    IChannel * channel = *channelResult;

    // -----------------------------
    // Create LIDAR driver
    // -----------------------------
    auto lidarResult = createLidarDriver();
    if (!lidarResult) {
        std::fprintf(stderr, "Error: cannot create LIDAR driver (insufficient memory?)\n");
        delete channel;
        return -1;
    }

    ILidarDriver * drv = *lidarResult;

    // Connect
    //sl_result res = drv->connect(*channel);
    sl_result res = drv->connect(channel);
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "Error: cannot connect to LIDAR (error code: %08x)\n", res);
        delete drv;
        delete channel;
        return -1;
    }

    // Start motor (for A1/A2/A3; S1/S2 sometimes don’t need this but it doesn’t hurt)
    drv->startMotor();
 
    // Start scan
    LidarScanMode scanMode;
    res = drv->startScan(false, true, 0, &scanMode);
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "Error: startScan failed (error code: %08x)\n", res);
        drv->stopMotor();
        drv->disconnect();
        delete drv;
        delete channel;
        return -1;
    }

    // -----------------------------
    // Initialize SDL2
    // -----------------------------
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        drv->stop();
        drv->stopMotor();
        drv->disconnect();
        delete drv;
        delete channel;
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "RPLIDAR Viewer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow error: %s\n", SDL_GetError());
        SDL_Quit();
        drv->stop();
        drv->stopMotor();
        drv->disconnect();
        delete drv;
        delete channel;
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        drv->stop();
        drv->stopMotor();
        drv->disconnect();
        delete drv;
        delete channel;
        return -1;
    }

    bool running = true;
    SDL_Event event;

    // -----------------------------
    // Render loop
    // -----------------------------
    while (running) {
        // Handle window close etc.
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        sl_result ans = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(ans)) {
            drv->ascendScanData(nodes, count);

            // Clear screen (black)
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);

            // Draw each lidar point (green)
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

            float cx = SCREEN_WIDTH / 2.0f;
            float cy = SCREEN_HEIGHT / 2.0f;

            for (size_t pos = 0; pos < count; ++pos) {
                float angle_deg = nodes[pos].angle_z_q14 * 90.0f / 16384.0f;
                float dist_mm   = nodes[pos].dist_mm_q2   / 4.0f;

                if (dist_mm <= 0.0f) continue;

                float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

                // Scale distance to fit screen radius
                float scaled = (dist_mm / static_cast<float>(MAX_DISTANCE_MM)) * (SCREEN_WIDTH / 2.0f);

                float x = cx + scaled * std::cos(angle_rad);
                float y = cy + scaled * std::sin(angle_rad);

                SDL_RenderDrawPoint(renderer, static_cast<int>(x), static_cast<int>(y));
            }

            SDL_RenderPresent(renderer);
        } else {
            // If we fail to grab data, don’t crash – just continue
            // You could add a small delay here if needed
        }
    }

    // -----------------------------
    // Cleanup
    // -----------------------------
    drv->stop();
    drv->stopMotor();
    drv->disconnect();

    delete drv;
    delete channel;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
