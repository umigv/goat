#include <phspat.h>

#include <csignal>
#include <atomic>
#include <iostream>
#include <mutex>

std::atomic<bool> keep_running{ true };
std::mutex cout_mtx;

void terminate_handler(int) {
    keep_running.store(false);
}

std::ostream& operator<<(std::ostream &os, const std::array<double, 3> &arr) {
    return os << "{ " << arr[0] << ", " << arr[1] << ", " << arr[2] << " }";
}

void on_data(const ph::Spatial&, const std::array<double, 3> &lin,
             const std::array<double, 3> &ang, const std::array<double, 3> &mag,
             double timestamp) {
    const std::unique_lock<std::mutex> lck{ cout_mtx };

    std::cout << "timestamp: " << timestamp << ", linear: " << lin
        << ", angular: " << ang << ", magnetic: " << mag << '\n';
}

void notify_attached(ph::Spatial spatial) {
    const std::unique_lock<std::mutex> lck{ cout_mtx };

    spatial.data_interval() = 100;

    std::cout << "attached\n";
}

void notify_detached(ph::Spatial) {
    const std::unique_lock<std::mutex> lck{ cout_mtx };

    std::cout << "detached\n";
}

int main() {
    ph::Spatial spatial;

    spatial.set_data_handler(on_data);
    spatial.set_attach_handler({ notify_attached });
    spatial.set_detach_handler({ notify_detached });

    std::signal(SIGTERM, terminate_handler);

    while (keep_running.load()) { }
}
