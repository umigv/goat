#include "umigv_utilities/ranges.hpp"
#include "umigv_utilities/types.hpp"

#include <functional>
#include <iostream>
#include <iterator>
#include <random>
#include <utility>
#include <vector>

using namespace umigv::types;

void map_enumerate_test() {
    std::cout << "map_enumerate_test" << std::endl;

    std::vector<f64> x{ 0.216, 0.786, 0.611, 0.641 };

    std::vector<f64> y;
    y.reserve(x.size());

    auto filter = [](const std::tuple<f64, const f64&> input) {
        static const f64 *last = nullptr;

        f64 output = std::get<1>(input);

        if (last) {
            output -= *last;
        }

        output *= std::get<0>(input);

        last = &std::get<1>(input);

        return output;
    };

    // y[n] = n * (x[n] - x[n - 1])
    for (auto &&i : umigv::map(umigv::enumerate<f64>(x), filter)) {
        y.push_back(i);
    }

    std::cout << "x = " << std::endl;

    for (const f64 value : x) {
        std::cout << '\t' << value << std::endl;
    }

    std::cout << "y = " << std::endl;

    for (const f64 value : y) {
        std::cout << '\t' << value << std::endl;
    }

    std::cout << std::endl;
}

void map_reduce_test() {
    std::cout << "map_reduce_test" << std::endl;

    std::vector<std::string> strings{ "foo", "bar", "baz" };

    // total length of all strings
    const usize length = umigv::reduce(umigv::map(strings, &std::string::size),
                                       std::plus<usize>{ });
    std::cout << "length = " << length << std::endl;

    const std::string concatenated = umigv::reduce(strings,
                                                   std::plus<std::string>{ });
    std::cout << "concatenated = " << concatenated << std::endl;

    std::cout << std::endl;
}

// prints the length of all strings from std::cin that are valid doubles
// according to std::stod
void map_filter_test() {
    std::cout << "map_filter_test" << std::endl;

    std::istream_iterator<std::string> begin{ std::cin };
    std::istream_iterator<std::string> end;

    auto &&range = umigv::iterator_range(std::move(begin), std::move(end));

    auto is_valid_number = [](const std::string &maybe_number) {
        try {
            __attribute__((unused)) const f64 num = std::stod(maybe_number);
            return true;
        } catch (...) {
            return false;
        }
    };

    auto string_and_size = [](const std::string &number) {
        return std::pair<usize, const std::string&>{ number.size(), number };
    };

    for (auto &&pair : umigv::map(umigv::filter(range,
                                                std::move(is_valid_number)),
                                  std::move(string_and_size))) {
        const usize length = pair.first;
        const std::string &string = pair.second;

        std::cout << string << ": " << length << std::endl;
    }

    std::cout << std::endl;
}

void map_zip_test() {
    std::cout << "map_zip_test" << std::endl;

    std::mt19937_64 engine;
    std::uniform_real_distribution<f64> distribution{ -1, 1 };

    // two vectors of eight random real numbers
    std::vector<f64> x(8);
    std::vector<f64> y(x.size());
    
    for (auto &&pair : umigv::zip(x, y)) {
        f64 &first = std::get<0>(pair);
        f64 &second = std::get<1>(pair);

        first = distribution(engine);
        second = distribution(engine);
    }

    std::cout << "x = " << std::endl;

    for (const f64 value : x) {
        std::cout << '\t' << value << std::endl;
    }

    std::cout << std::endl
              << "y = " << std::endl;

    for (const f64 value : y) {
        std::cout << '\t' << value << std::endl;
    }

    std::cout << std::endl;

    auto sum_return = [](std::tuple<f64&, f64&, f64&> tuple) {
        return std::pair<f64, f64&>{ std::get<0>(tuple) + std::get<1>(tuple),
                                     std::get<2>(tuple) };
    };

    std::vector<f64> z(x.size());

    for (auto &&sum : umigv::map(umigv::zip(x, y, z), std::move(sum_return))) {
        const f64 first = sum.first;
        f64 &second = sum.second;

        second = first;
    }

    std::cout << "z = " << std::endl;

    for (const f64 value : z) {
        std::cout << '\t' << value << std::endl;
    }

    std::cout << std::endl;

    const f64 total = umigv::reduce(z, std::plus<f64>{ });

    std::cout << "total = " << total << std::endl
              << std::endl;
}

int main() {
    map_enumerate_test();
    map_reduce_test();
    map_zip_test();
    map_filter_test();
}
