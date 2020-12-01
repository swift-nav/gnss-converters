#ifndef NOVATEL_PARSER_MESSAGE_INSATT_H_
#define NOVATEL_PARSER_MESSAGE_INSATT_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct INSATT_t {
  uint32_t gnss_week;
  double tow;
  double roll;
  double pitch;
  double azimuth;
  uint32_t status;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
