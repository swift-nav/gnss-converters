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

#ifndef NOVATEL_PARSER_MESSAGE_H_
#define NOVATEL_PARSER_MESSAGE_H_

#include <array>
#include <functional>

#include "binary_header.h"

namespace Novatel {
namespace Message {

enum Id {
  GPSEPHEM = 7,
  BESTPOS = 42,
  BESTVEL = 99,
  RANGECMP = 140,
  INSATT = 263,
  GLOEPHEMERIS = 723,
  RAWIMUSX = 1462,
  // Be sure to update this when adding more messages.
  kNumMessageIds = 7,
  kNoCallback = -1,
};

/**
 * Callback
 * --------
 * Users can supply a callback which will be called when a message is
 * successfully parsed. Note that the function is responsible for correctly
 * casting the body data pointer to the type which matches the message_id. The
 * data available through the header and body pointers is only guaranteed to be
 * valid for the scope of the function call (and not across threads).
 *
 * e.g.:
 *
 * Callback mycallback;
 * mycallback.message_id = Id::BESTPOS;
 * mycallback.callfn = [&](const BinaryHeader *header, const void *body) {
 *      printf("My callback!\n");
 * };
 */
struct Callback {
  Id message_id;
  std::function<void(const BinaryHeader *, const void *)> callfn;
};

using CallbackArray = std::array<Callback, kNumMessageIds>;

}  // namespace Message
}  // namespace Novatel

#endif
