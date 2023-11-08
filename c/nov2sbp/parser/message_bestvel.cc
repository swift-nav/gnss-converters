#include "message_bestvel.h"

#include <cassert>

#include "read_little_endian.h"

namespace Novatel {

void Message::BESTVEL_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;
  sol_stat = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  vel_type = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  latency = Util::read_le_float(bytes + i);
  i += sizeof(float);
  diff_age = Util::read_le_float(bytes + i);
  i += sizeof(float);
  hor_speed = Util::read_le_double(bytes + i);
  i += sizeof(double);
  trk_gnd = Util::read_le_double(bytes + i);
  i += sizeof(double);
  vert_speed = Util::read_le_double(bytes + i);
  i += sizeof(double);
  reserved = Util::read_le_float(bytes + i);
  i += sizeof(float);

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
