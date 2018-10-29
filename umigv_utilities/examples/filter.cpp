#include "umigv_utilities/filter.hpp"
#include "umigv_utilities/types.hpp"

#include <iostream>
#include <string>
#include <vector>

using namespace umigv::types;

// print only nonempty strings
void string_test() {
    std::cout << "string_test" << std::endl;

    std::vector<std::string> strings{ "", "foo", "bar", "baz" };

    for (const std::string &string : umigv::filter(strings,
                                                   &std::string::size)) {
        std::cout << string << std::endl;
    }

    std::cout << std::endl;
}

// collect only odd numbers
void i32_test() {
    std::cout << "i32_test" << std::endl;

    std::vector<i32> numbers;

    for (const i32 num : umigv::filter({ 0, 1, 2, 3, 4, 5, 6 },
                                       [](const i32 x) { return x % 2 == 1; })) {
        numbers.push_back(num);
    }

    std::cout << "numbers =" << std::endl;

    for (const i32 num : numbers) {
        std::cout << '\t' << num << std::endl;
    }

    std::cout << std::endl;
}

int main() {
    string_test();
    i32_test();
}
