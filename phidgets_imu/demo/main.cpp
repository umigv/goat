#include <phspat.h>

#include <csignal>
#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

std::atomic<bool> keep_running{ true };
std::mutex cout_mtx;

void terminate_handler(int) {
    keep_running.store(false);
}

std::ostream& operator<<(std::ostream &os, const ph::spatial::Vector &vec) {
    return os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
}

struct OnData {
    void operator()(ph::Spatial, const ph::spatial::Data &data) const {
        const std::unique_lock<std::mutex> lck{ cout_mtx };
        std::cout << "{\n"
                  << "    \"timestamp\": " << data.timestamp.time_since_epoch().count() << ",\n"
                  << "    \"linear_acceleration\": " << data.linear_acceleration << ",\n"
                  << "    \"angular_velocity\": " << data.angular_velocity << ",\n"
                  << "    \"magnetic_field\": " << data.magnetic_field << '\n'
                  << "},\n";
    }
};

struct OnAttach {
    void operator()(ph::Spatial spatial) const {
        spatial.data_interval() = spatial.min_data_interval();

        const std::unique_lock<std::mutex> lck{ cout_mtx };
        std::cerr << "attached\n";
    }
};

int main() {
    ph::Spatial spatial{ OnData{ }, OnAttach{ } };

    std::signal(SIGTERM, terminate_handler);

    while (keep_running.load()) { }
}
