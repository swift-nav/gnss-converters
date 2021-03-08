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

#ifndef NOVATEL_PARSER_PARSER_H_
#define NOVATEL_PARSER_PARSER_H_

#include <unistd.h>  // for size_t
#include "binary_header.h"
#include "message.h"

namespace Novatel {

class Parser {
 public:
  /**
   * This function should attempt to populate the uint8_t buffer
   * with size_t bytes. The number of bytes successfully read should be
   * returned. To indicate error the function should return -1.
   */
  typedef int (*ReadFunction)(uint8_t *, uint32_t, void *);

  /**
   * Optional function to log warning messages
   */
  typedef void (*LogFunction)(const char *);

  /**
   * Initialize with a byte-reading-function, and an array of callbacks to be
   * called when specific message are parsed.
   */
  Parser(ReadFunction readfn,
         LogFunction logfn,
         const Message::CallbackArray &callbacks,
         void *context);

  /**
   * Parse input
   */
  void process();

 private:
  /**
   * Long enough to hold either a header (up to 256 bytes),
   * or a message body. Techically a message body can be up
   * to 65354 bytes, but in this implementation we limit it
   * to a more reasonable amount which seems to be long enough
   * in practice.
   */
  static constexpr size_t kBufferSize = 4096;
  uint8_t buffer_[kBufferSize];
  uint32_t num_bytes_in_buffer_;

  ReadFunction readfn_;
  LogFunction logfn_;

  const Message::CallbackArray callbacks_;

  void *context_;

  uint32_t parse_regular_message() const;
  uint32_t parse_short_message() const;
  void parse_body(const BinaryHeader &header, const uint8_t *data) const;
  void invoke_callback(const BinaryHeader &header, const void *data) const;
  void warn(const char *format, ...) const
      __attribute__((format(printf, 2, 3)));
};

};  // namespace Novatel

#endif
