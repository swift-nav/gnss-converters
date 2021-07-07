#ifndef LIBUBX_PRIVATE_DECODE_HELPERS_H
#define LIBUBX_PRIVATE_DECODE_HELPERS_H

#define BYTESTREAM_DECODE_BYTES(bytestream, field, n_bytes)     \
  do {                                                          \
    assert((n_bytes) == sizeof(field));                         \
    if (!swiftnav_bytestream_get_bytes(                         \
            (bytestream), 0, (n_bytes), (uint8_t *)&(field))) { \
      return RC_INVALID_MESSAGE;                                \
    }                                                           \
    swiftnav_bytestream_remove((bytestream), (n_bytes));        \
  } while (false)

#endif
