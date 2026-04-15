#include <cstring>

// Match the signature expected by ProbeCommVariant.cpp
extern "C"
{

struct SharedMemReadResult
{
    char value[6];
    bool success;
};

SharedMemReadResult Icmem_ReadVariantCode()
{
    SharedMemReadResult result{};
    std::strncpy(result.value, "00000", 5);
    result.value[5] = '\0';
    result.success = false;   // Force fallback path
    return result;
}

} // extern "C"