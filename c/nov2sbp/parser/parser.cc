/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "parser.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "crc_checker.h"
#include "message_bestpos.h"
#include "message_bestvel.h"
#include "message_gloephemeris.h"
#include "message_gpsephem.h"
#include "message_insatt.h"
#include "message_rangecmp.h"
#include "message_rawimusx.h"
#include "read_little_endian.h"

namespace Novatel {

// Standard Binary Message described in Chapter 1.3 of the
// OEM7 Commands and Logs Reference Manual
static constexpr uint8_t MSG_REGULAR_PREAMBLE[3] = {0xAA, 0x44, 0x12};

// header length (including preamble)
static constexpr int MSG_REGULAR_HEADER_LEN = 28;

// Short Binary Message described in Chapter 1.4 of the
// OEM7 Commands and Logs Reference Manual
static constexpr uint8_t MSG_SHORT_PREAMBLE[3] = {0xAA, 0x44, 0x13};

// header length (including preamble)
static constexpr int MSG_SHORT_HEADER_LEN = 12;

// CRC used for both regular and short binary messages
static constexpr int MSG_CRC_LEN = 4;

Parser::Parser(ReadFunction readfn,
               LogFunction logfn,
               const Message::CallbackArray &callbacks,
               void *context)
    : num_bytes_in_buffer_(0),
      readfn_(readfn),
      logfn_(logfn),
      callbacks_{callbacks},
      context_(context) {}

void Parser::process() {
  do {
    // fill buffer_
    int num_bytes_read = readfn_(buffer_ + num_bytes_in_buffer_,
                                 sizeof(buffer_) - num_bytes_in_buffer_,
                                 context_);

    if (num_bytes_read < 0) {
      warn("Read error");
      return;
    }

    num_bytes_in_buffer_ += num_bytes_read;
    uint32_t num_bytes_consumed = 1;

    if (num_bytes_in_buffer_ >= sizeof(MSG_REGULAR_PREAMBLE)) {
      // check for regular preamble (0xAA, 0x44, 0x12)
      if (0 ==
          memcmp(buffer_, MSG_REGULAR_PREAMBLE, sizeof(MSG_REGULAR_PREAMBLE))) {
        num_bytes_consumed = parse_regular_message();
        // check for short preamble (0xAA, 0x44, 0x13)
      } else if (0 == memcmp(buffer_,
                             MSG_SHORT_PREAMBLE,
                             sizeof(MSG_SHORT_PREAMBLE))) {
        num_bytes_consumed = parse_short_message();
      }
    }

    // drop consumed bytes from buffer_
    assert(num_bytes_consumed <= num_bytes_in_buffer_);
    num_bytes_in_buffer_ -= num_bytes_consumed;
    memmove(buffer_, buffer_ + num_bytes_consumed, num_bytes_in_buffer_);
  } while (num_bytes_in_buffer_ > 0);
}

/*
 * Attempt to parse buffer_ as a regular Novatel Binary Message, return number
 * of bytes consumed
 */
uint32_t Parser::parse_regular_message() const {
  if (num_bytes_in_buffer_ >= MSG_REGULAR_HEADER_LEN + MSG_CRC_LEN) {
    // header length is in byte following preamble
    if (buffer_[sizeof(MSG_REGULAR_PREAMBLE)] == MSG_REGULAR_HEADER_LEN) {
      BinaryHeaderRegular header;
      // skip extra byte since 'header' does not include header length
      header.read(buffer_ + sizeof(MSG_REGULAR_PREAMBLE) + 1);

      if (header.message_len <=
          num_bytes_in_buffer_ - MSG_REGULAR_HEADER_LEN - MSG_CRC_LEN) {
        uint32_t crc_expected = Util::read_le_uint32(
            &buffer_[MSG_REGULAR_HEADER_LEN + header.message_len]);
        uint32_t crc_calculated = CrcChecker::get_crc(
            buffer_, MSG_REGULAR_HEADER_LEN + header.message_len);
        if (crc_expected == crc_calculated) {
          parse_body(header, buffer_ + MSG_REGULAR_HEADER_LEN);
          return MSG_REGULAR_HEADER_LEN + MSG_CRC_LEN + header.message_len;
        }
        warn("Unexpected CRC %.8x vs %.8x", crc_expected, crc_calculated);
      } else {
        warn("Unexpectedly large message length %i", header.message_len);
      }
    } else {
      warn("Unexpected header length %i",
           buffer_[sizeof(MSG_REGULAR_PREAMBLE)]);
    }
  }

  // parsing failed, drop 1 byte and try again
  return 1;
}

/*
 * Attempt to parse buffer_ as a short Novatel Binary Message, return number
 * of bytes consumed
 */
uint32_t Parser::parse_short_message() const {
  if (num_bytes_in_buffer_ >= MSG_SHORT_HEADER_LEN + MSG_CRC_LEN) {
    BinaryHeaderShort header;
    header.read(buffer_ + sizeof(MSG_SHORT_PREAMBLE));

    if (header.message_len <=
        num_bytes_in_buffer_ - MSG_SHORT_HEADER_LEN - MSG_CRC_LEN) {
      uint32_t crc_expected = Util::read_le_uint32(
          &buffer_[MSG_SHORT_HEADER_LEN + header.message_len]);
      uint32_t crc_calculated = CrcChecker::get_crc(
          buffer_, MSG_SHORT_HEADER_LEN + header.message_len);

      if (crc_expected == crc_calculated) {
        parse_body(header, buffer_ + MSG_SHORT_HEADER_LEN);
        return MSG_SHORT_HEADER_LEN + MSG_CRC_LEN + header.message_len;
      }
      warn("Unexpected CRC %.8x vs %.8x", crc_expected, crc_calculated);
    } else {
      warn("Unexpectedly large message length %i", header.message_len);
    }
  }

  // parsing failed, drop 1 byte and try again
  return 1;
}

/**
 * This function dispatches to the appropriate body parser depending on the
 * message ID.
 */
void Parser::parse_body(const BinaryHeader &header, const uint8_t *data) const {
  switch (header.message_id) {
    case Message::Id::GPSEPHEM: {
      Message::GPSEPHEM_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::BESTPOS: {
      Message::BESTPOS_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::BESTVEL: {
      Message::BESTVEL_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::RANGECMP: {
      // This is a variable length message so we must first read the number of
      // records.
      Message::RANGECMP_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::INSATT: {
      Message::INSATT_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::GLOEPHEMERIS: {
      Message::GLOEPHEMERIS_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    case Message::Id::RAWIMUSX: {
      Message::RAWIMUSX_t msg;
      msg.FromBytes(data, header.message_len);
      invoke_callback(header, &msg);
      break;
    }
    default:
      // assert(0 && "Unsupported message type.");
      break;
  }
}

/**
 * Pass the data along to the appropriate callback function.
 */
void Parser::invoke_callback(const BinaryHeader &header,
                             const void *data) const {
  for (const auto &callback : callbacks_) {
    if (callback.message_id == header.message_id && callback.callfn) {
      callback.callfn(&header, data);
    }
  }
}

/*
 * Stringify the message and send to logfn_
 */
void Parser::warn(const char *format, ...) const {  // NOLINT
  if (logfn_ == nullptr) {
    return;
  }

  char buf[256];
  va_list ap;

  va_start(ap, format);
  // clang-tidy warns on this, SO claims it's a bug:
  // https://stackoverflow.com/a/58681310/749342
  vsnprintf(buf, sizeof(buf), format, ap);  // NOLINT
  va_end(ap);

  logfn_(buf);
}
}  // namespace Novatel
