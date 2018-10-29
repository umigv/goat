#include "umigv_utilities/enumerate.hpp"
#include "umigv_utilities/types.hpp"

#include <iostream>
#include <iterator>
#include <unordered_map>
#include <vector>

using namespace umigv::types;

void vector_test() {
    std::vector<i32> numbers(4);

    for (auto &&iv : umigv::enumerate<i32>(numbers)) {
        const i32 index = iv.first;
        i32 &value = iv.second;

        value = index % 2;
    }

    for (const i32 number : numbers) {
        std::cout << number << std::endl;
    }
}

void map_test() {
    std::unordered_map<std::string, i32> friend_count;

    friend_count["alice"] = 10;
    friend_count["bob"] = 5;
    friend_count["catherine"] = 15;

    for (auto &&ikv : umigv::enumerate(friend_count)) {
        const usize index = ikv.first;
        const std::string &key = ikv.second.first;
        const i32 &value = ikv.second.second;

        std::cout << index << ": [" << key << ", " << value << "]" << std::endl;
    }
}

void istream_test() {
    std::istream_iterator<i32> begin{ std::cin };
    std::istream_iterator<i32> end;

    for (auto &&iv : umigv::enumerate(begin, end)) {
        const usize index = iv.first;
        const i32 value = iv.second;

        std::cout << index << ": " << value << std::endl;
    }
}

int main() {
    vector_test();

    std::cout << std::endl;

    map_test();

    std::cout << std::endl;

    istream_test();
}
