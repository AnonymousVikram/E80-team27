#include "FloatFormatter.h"

FloatFormatter::FloatFormatter(void) {
  data << std::fixed << std::setprecision(3);
}

std::string FloatFormatter::format(float value) {
  data << value << ",";
  std::string returner = data.str();
  data.str("");
  return returner;
}