#pragma once

#include "module/functions.h"

inline float sbus_range(int16_t x) {
    return mapf(x,224.,1759.,-1.,1.);
}

enum tri_switch {
    idle,
    middle,
    active
};

inline tri_switch sbus_switch(int16_t x) {
    if (x < 736 ) return tri_switch::idle;
    else if (x > 1248 ) return tri_switch::active;
    else return tri_switch::middle;
}