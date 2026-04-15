#include <cstdint>

extern "C"
{

int8_t Icmem_ReadVariantCode(uint8_t* out, uint8_t len)
{
    // Safe fallback: fill with '0'
    for (uint8_t i = 0U; i < len; ++i)
    {
        out[i] = static_cast<uint8_t>('0');
    }
    return -1; // indicate failure, triggers safe defaults
}

}