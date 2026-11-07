#include "bbcp/utils/MathUtils.h"

namespace bbcp {

int64_t CeilDivPositive(int64_t a, int64_t b) {
    if (a <= 0) return 0;
    return (a + b - 1) / b;
}

}  // namespace bbcp
