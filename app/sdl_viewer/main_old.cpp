#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace {

constexpr int kWindowWidth = 900;
constexpr int kWindowHeight = 900;
constexpr float kDefaultRangeMm = 6000.0f;
constexpr float kMarginPixels = 24.0f;

float angleToRadians(const sl_lidar_response_measurement_node_hq_t &node) {
    return static_cast<float>(node.angle_z_q14) * static_cast<float>(M_PI) / 32768.0f;
}

float distanceMillimeters(const sl_lidar_response_measurement_node_hq_t &node) {
    return static_cast<float>(node.dist_mm_q2) / 4.0f;
}

class ChannelHandle {
public:
    ChannelHandle() = default;
    explicit ChannelHandle(IChannel *channel) : channel_(channel) {}
    ChannelHandle(const ChannelHandle &) = delete;
    ChannelHandle &operator=(const ChannelHandle &) = delete;
    ChannelHandle(ChannelHandle &&other) noexcept : channel_(other.channel_) { other.channel_ = nullptr; }
    ChannelHandle &operator=(ChannelHandle &&other) noexcept {
        if (this != &other) {
            reset();
            channel_ = other.channel_;
            other.channel_ = nullptr;
        }
        return *this;
    }
    ~ChannelHandle() { reset(); }

    IChannel *get() const { return channel_; }
    explicit operator bool() const { return channel_ != nullptr; }

private:
    void reset() {
        if (channel_) {
            delete channel_;
            channel_ = nullptr;
        }
    }

    IChannel *channel_ = nullptr;
};

class DriverHandle {
public:
    DriverHandle() = default;
    explicit DriverHandle(ILidarDriver *driver) : driver_(driver) {}
    DriverHandle(const DriverHandle &) = delete;
    DriverHandle &operator=(const DriverHandle &) = delete;
    DriverHandle(DriverHandle &&other) noexcept : driver_(other.driver_) { other.driver_ = nullptr; }
    DriverHandle &operator=(DriverHandle &&other) noexcept {
        if (this != &other) {
            reset();
            driver_ = other.driver_;
            other.driver_ = nullptr;
        }
        return *this;
    }
    ~DriverHandle() { reset(); }

    ILidarDriver *get() const { return driver_; }
    ILidarDriver *operator->() const { return driver_; }
    explicit operator bool() const { return driver_ != nullptr; }

private:
    void reset() {
        if (driver_) {
            driver_->stop();
            driver_->disconnect();
            delete driver_;
            driver_ = nullptr;
        }
    }

    ILidarDriver *driver_ = nullptr;
};

struct SDLContext {
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
};

bool initializeSDL(SDLContext &context) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    context.window = SDL_CreateWindow("RPLIDAR SDL Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                      kWindowWidth, kWindowHeight, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!context.window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return false;
    }

    context.renderer = SDL_CreateRenderer(context.window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!context.renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(context.window);
        SDL_Quit();
        return false;
    }

    return true;
}

void shutdownSDL(SDLContext &context) {
    if (context.renderer) {
        SDL_DestroyRenderer(context.renderer);
        context.renderer = nullptr;
    }
    if (context.window) {
        SDL_DestroyWindow(context.window);
        context.window = nullptr;
    }
    SDL_Quit();
}

void drawRing(SDL_Renderer *renderer, const SDL_Point &center, float radius, SDL_Color color) {
    constexpr int kSegments = 240;
    std::vector<SDL_Point> points(kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
        const float theta = static_cast<float>(i) * 2.0f * static_cast<float>(M_PI) / static_cast<float>(kSegments);
        points[i].x = static_cast<int>(center.x + radius * std::cos(theta));
        points[i].y = static_cast<int>(center.y - radius * std::sin(theta));
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLines(renderer, points.data(), static_cast<int>(points.size()));
}

void renderScan(SDL_Renderer *renderer, const SDL_Point &center, const std::vector<sl_lidar_response_measurement_node_hq_t> &nodes,
                float displayRangeMm) {
    int windowWidth = 0;
    int windowHeight = 0;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    const float availableRadius = 0.5f * static_cast<float>(std::min(windowWidth, windowHeight)) - kMarginPixels;
    const float mmToPixels = availableRadius / displayRangeMm;

    SDL_SetRenderDrawColor(renderer, 10, 10, 10, 255);
    SDL_RenderClear(renderer);

    SDL_Color gridColor{60, 60, 60, 255};
    for (float ringMm = 1000.0f; ringMm <= displayRangeMm; ringMm += 1000.0f) {
        drawRing(renderer, center, ringMm * mmToPixels, gridColor);
    }

    SDL_SetRenderDrawColor(renderer, 120, 120, 120, 255);
    SDL_RenderDrawLine(renderer, 0, center.y, windowWidth, center.y);
    SDL_RenderDrawLine(renderer, center.x, 0, center.x, windowHeight);

    SDL_SetRenderDrawColor(renderer, 46, 204, 113, 255);
    for (const auto &node : nodes) {
        const float distanceMm = distanceMillimeters(node);
        if (distanceMm <= 0.0f || node.quality == 0) {
            continue;
        }
        const float angle = angleToRadians(node);
        const float radius = std::min(distanceMm, displayRangeMm) * mmToPixels;
        const int x = static_cast<int>(std::lround(center.x + radius * std::cos(angle)));
        const int y = static_cast<int>(std::lround(center.y - radius * std::sin(angle)));
        SDL_RenderDrawPoint(renderer, x, y);
    }

    SDL_RenderPresent(renderer);
}

bool parseArguments(int argc, const char *argv[], std::string &port, sl_u32 &baudrate) {
    port = "/dev/ttyUSB0";
    baudrate = 115200;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baudrate = static_cast<sl_u32>(std::strtoul(argv[++i], nullptr, 10));
        } else if (arg == "--help") {
            std::printf("Usage: %s [--port /dev/ttyUSB0] [--baud 115200]\n", argv[0]);
            return false;
        }
    }
    return true;
}

bool retrieveScan(ILidarDriver *driver, std::vector<sl_lidar_response_measurement_node_hq_t> &buffer) {
    size_t count = buffer.size();
    const sl_result result = driver->grabScanDataHq(buffer.data(), count);
    if (!SL_IS_OK(result) && result != SL_RESULT_OPERATION_TIMEOUT) {
        std::fprintf(stderr, "Failed to grab scan data: %x\n", result);
        return false;
    }

    if (SL_IS_OK(result)) {
        driver->ascendScanData(buffer.data(), count);
        buffer.resize(count);
    } else {
        buffer.resize(0);
    }
    return true;
}

}  // namespace

int main(int argc, const char *argv[]) {
    std::string port;
    sl_u32 baudrate = 0;
    if (!parseArguments(argc, argv, port, baudrate)) {
        return 0;
    }

    auto channelResult = createSerialPortChannel(port, baudrate);
    if (!channelResult) {
        std::fprintf(stderr, "Unable to create serial channel on %s\n", port.c_str());
        return 1;
    }
    ChannelHandle channel(*channelResult);

    auto driverResult = createLidarDriver();
    if (!driverResult) {
        std::fprintf(stderr, "Unable to create RPLIDAR driver\n");
        return 1;
    }
    DriverHandle driver(*driverResult);

    const sl_result connectResult = driver->connect(channel.get());
    if (!SL_IS_OK(connectResult)) {
        std::fprintf(stderr, "Failed to connect to RPLIDAR on %s (%x)\n", port.c_str(), connectResult);
        return 1;
    }

    (void)driver->setMotorSpeed();

    LidarScanMode scanMode;
    const sl_result startResult = driver->startScan(false, true, 0, &scanMode);
    if (!SL_IS_OK(startResult)) {
        std::fprintf(stderr, "Failed to start scan (%x)\n", startResult);
        return 1;
    }

    SDLContext sdl;
    if (!initializeSDL(sdl)) {
        return 1;
    }

    SDL_Point center{kWindowWidth / 2, kWindowHeight / 2};
    std::vector<sl_lidar_response_measurement_node_hq_t> nodes(8192);
    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
                center.x = event.window.data1 / 2;
                center.y = event.window.data2 / 2;
            }
        }

        if (!retrieveScan(driver.get(), nodes)) {
            break;
        }

        float maxDistance = kDefaultRangeMm;
        for (const auto &node : nodes) {
            const float distanceMm = distanceMillimeters(node);
            if (distanceMm > maxDistance) {
                maxDistance = distanceMm;
            }
        }

        renderScan(sdl.renderer, center, nodes, maxDistance);
    }

    shutdownSDL(sdl);
    return(0); 
}