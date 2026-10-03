#include "document_fuzz.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    // libFuzzer owns this byte buffer for the duration of the call.
    const auto input = std::string_view(reinterpret_cast<const char*>(data), size);
    (void)rigidbodies::fuzz::exercise_document(input);
    return 0;
}
