#pragma once

namespace P64::Coll {
    constexpr float DEFAULT_GFX_SCALE = 100.0f;

    inline float gfxScale_{DEFAULT_GFX_SCALE};
    inline float inverseGfxScale_{1.0f / DEFAULT_GFX_SCALE};

    inline float getGfxScale() { return gfxScale_; }
    inline float getInvGfxScale() { return inverseGfxScale_; }

    inline void setGfxScale(float gfxScale) {
        gfxScale_ = gfxScale;
        inverseGfxScale_ = 1.0f / gfxScale;
    }
}
