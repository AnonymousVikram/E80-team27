#ifndef FLOAT_FORMATTER_H_INCLUDED
#define FLOAT_FORMATTER_H_INCLUDED

#include <iomanip>
#include <sstream>
#include <string>

class FloatFormatter {
public:
  FloatFormatter(void);

  std::string format(float value);

private:
  std::stringstream data;
};

#endif