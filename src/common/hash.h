#ifndef WIFLX_COMMON_HASH_H
#define WIFLX_COMMON_HASH_H

#include <cstdint>
#include <cstdlib>

namespace wiflx {
namespace common {

extern uint32_t hashlittle (const void *key, size_t length, uint32_t initval);

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_HASH_H
