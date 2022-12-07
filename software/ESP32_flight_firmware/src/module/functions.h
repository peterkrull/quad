#pragma once

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    const float out_delta = out_max - out_min;
    const float in_delta = in_max - in_min;

    return ((x - in_min)/in_delta)*out_delta + out_min;
}