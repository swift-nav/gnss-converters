#ifndef NOVATEL_PARSER_MESSAGE_BESTVEL_H_
#define NOVATEL_PARSER_MESSAGE_BESTVEL_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct BESTVEL_t {
  uint32_t sol_stat;
  uint32_t vel_type;
  float latency;
  float diff_age;
  double hor_speed;
  double trk_gnd;
  double vert_speed;
  float reserved;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
