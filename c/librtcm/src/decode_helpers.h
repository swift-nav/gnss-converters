#ifndef RTCM_PRIVATE_DECODE_HELPERS_H
#define RTCM_PRIVATE_DECODE_HELPERS_H

#define BITSTREAM_DECODE_BOOL(bitstream, field, n_bits)        \
  do {                                                         \
    u32 decode_bool_temp;                                      \
    BITSTREAM_DECODE_U32(bitstream, decode_bool_temp, n_bits); \
    (field) = (bool)decode_bool_temp;                          \
  } while (false)

#define BITSTREAM_DECODE_U64(bitstream, field, n_bits)                         \
  do {                                                                         \
    u64 decode_u64_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(u64)) ? 1 : -1];           \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbitul(                                          \
            bitstream, &decode_u64_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_u64_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_S64(bitstream, field, n_bits)                         \
  do {                                                                         \
    s64 decode_s64_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(s64)) ? 1 : -1];           \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbitsl(                                          \
            bitstream, &decode_s64_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_s64_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_U32(bitstream, field, n_bits)                         \
  do {                                                                         \
    u32 decode_u32_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(u32)) ? 1 : -1];           \
    char /* NOLINTNEXTLINE(clang-analyzer-core.VLASize) */                     \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbitu(bitstream, &decode_u32_temp, 0, n_bits)) { \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_u32_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_S32(bitstream, field, n_bits)                         \
  do {                                                                         \
    s32 decode_s32_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(s32)) ? 1 : -1];           \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbits(bitstream, &decode_s32_temp, 0, n_bits)) { \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_s32_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_U16(bitstream, field, n_bits)                         \
  do {                                                                         \
    u32 decode_u16_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(u16)) ? 1 : -1];           \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbitu(bitstream, &decode_u16_temp, 0, n_bits)) { \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_u16_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_S16(bitstream, field, n_bits)                         \
  do {                                                                         \
    s32 decode_s16_temp;                                                       \
    char __decode_assert_1[(sizeof(field) == sizeof(s16)) ? 1 : -1];           \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbits(bitstream, &decode_s16_temp, 0, n_bits)) { \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_s16_temp;                                                 \
  } while (false)

#define BITSTREAM_DECODE_U8(bitstream, field, n_bits)                          \
  do {                                                                         \
    u32 decode_u8_temp;                                                        \
    char __decode_assert_1[(sizeof(field) == sizeof(u8)) ? 1 : -1];            \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbitu(bitstream, &decode_u8_temp, 0, n_bits)) {  \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_u8_temp;                                                  \
  } while (false)

#define BITSTREAM_DECODE_S8(bitstream, field, n_bits)                          \
  do {                                                                         \
    s32 decode_s8_temp;                                                        \
    char __decode_assert_1[(sizeof(field) == sizeof(s8)) ? 1 : -1];            \
    char                                                                       \
        __decode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__decode_assert_1;                                                   \
    (void)__decode_assert_2;                                                   \
    if (!swiftnav_in_bitstream_getbits(bitstream, &decode_s8_temp, 0, n_bits)) {  \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_in_bitstream_remove(bitstream, n_bits);                              \
    (field) = decode_s8_temp;                                                  \
  } while (false)

/* macros for reading rcv/ant descriptor strings */
#define GET_STR_LEN(TheBuff, TheIdx, TheOutput)         \
  do {                                                  \
    (TheOutput) = rtcm_getbitu((TheBuff), (TheIdx), 8); \
    if (RTCM_MAX_STRING_LEN <= (TheOutput)) {           \
      return RC_INVALID_MESSAGE;                        \
    }                                                   \
    (TheIdx) += 8;                                      \
  } while (false);

#define GET_STR(TheBuff, TheIdx, TheLen, TheOutput)          \
  do {                                                       \
    for (uint8_t i = 0; i < (TheLen); ++i) {                 \
      (TheOutput)[i] = rtcm_getbitu((TheBuff), (TheIdx), 8); \
      (TheIdx) += 8;                                         \
    }                                                        \
  } while (false);

#endif
