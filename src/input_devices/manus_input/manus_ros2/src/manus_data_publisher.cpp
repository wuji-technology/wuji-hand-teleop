#include <memory>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"

#include "ManusDataPublisher.hpp"

static std::shared_ptr<ManusDataPublisher> g_node = nullptr;
static volatile sig_atomic_t g_shutdown_requested = 0;

void signalHandler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        g_shutdown_requested = 1;
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
}

int main(int argc, char* argv[])
{
    // Register signal handlers before initializing ROS
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::init(argc, argv);
    g_node = std::make_shared<ManusDataPublisher>();

    // Use executor with interruptible spinning
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(g_node);

    while (rclcpp::ok() && !g_shutdown_requested) {
        executor.spin_some(std::chrono::milliseconds(100));
    }

    // Ensure proper cleanup - destructor will be called
    g_node.reset();

    rclcpp::shutdown();
    return 0;
}
