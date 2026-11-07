#include <cassert>
#include <cmath>
#include "bbcp/master/LipschitzCuts.h"
int main() {
  bbcp::Instance inst;
  assert(bbcp::LipschitzCuts::MakespanDrop(inst, 1, 100) == 8);
  assert(bbcp::LipschitzCuts::MakespanDrop(inst, 1, 4) == 4);
  assert(bbcp::LipschitzCuts::MakespanDrop(inst, 1, 0) == 0);
  assert(bbcp::LipschitzCuts::MakespanDrop(inst, 2, 12) == 12);
}
