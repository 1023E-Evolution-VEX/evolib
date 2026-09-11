#pragma once

#include <stdint.h>

#include <cstddef>

#include "json.hpp"

using Json = json11::Json;

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
    uint8_t* buf;
    size_t size;
} asset;
}

Json open_asset_as_json(asset x);

#define ASSET(x)                                                              \
    extern "C" {                                                              \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[]; \
    static asset x = {_binary_static_##x##_start,                             \
                      (size_t)_binary_static_##x##_size};                     \
    }
