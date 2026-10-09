

#include <dimos/native/cdr_codec.hpp>
#include <demo_msgs/msg/telemetry.hpp>
#include <fstream>
#include <iostream>
#include <iterator>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::ifstream input(argv[1], std::ios::binary);
    std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), {}};
    auto value = dimos::native::cdr_decode<demo_msgs::msg::Telemetry>(bytes);
    std::cout << "Installed C++ package received: " << value.application_note << '\n';
    value.application_note += "/cpp";
    auto encoded = dimos::native::cdr_encode(value);
    std::ofstream output(argv[2], std::ios::binary);
    output.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
    return output ? 0 : 1;
}
