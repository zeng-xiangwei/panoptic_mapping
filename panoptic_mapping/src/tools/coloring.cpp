#include "panoptic_mapping/tools/coloring.h"

namespace panoptic_mapping { 
namespace {

constexpr float kInitialHue = 0.69f;
constexpr float kSaturation = 0.85f;
constexpr float kValue = 0.77f;

Color hsvToRgb(const float h, const float s, const float v) {
  const float h_6 = (h == 1.f) ? 0.f : 6 * h;
  const int h_i = std::floor(h_6);
  const float f = h_6 - h_i;

  const uint8_t p = static_cast<uint8_t>(v * (1.f - s) * 255);
  const uint8_t q = static_cast<uint8_t>(v * (1.f - f * s) * 255);
  const uint8_t t = static_cast<uint8_t>(v * (1.f - (1.f - f) * s) * 255);

  if (h_i == 0) {
    return {v, t, p};
  } else if (h_i == 1) {
    return {q, v, p};
  } else if (h_i == 2) {
    return {p, v, t};
  } else if (h_i == 3) {
    return {p, q, v};
  } else if (h_i == 4) {
    return {t, p, v};
  } else if (h_i == 5) {
    return {v, p, q};
  } else {
    return {0, 0, 0};
  }
}

}  // namespace

Color generateColor(int id) {
  // Uniform color sampling using the golden ratio from
  // http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
  constexpr float kGoldenRatioConjugate = 0.6180339887498949f;
  const float hue = std::fmod(kInitialHue + kGoldenRatioConjugate * id, 1.f);
  return hsvToRgb(hue, kSaturation, kValue);
}

}