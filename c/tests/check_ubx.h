/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef CHECK_UBX_H
#define CHECK_UBX_H

#include <libsbp/logging.h>

#include <ubx/decode.h>
#include <ubx/ubx_messages.h>

#define MAX_FILE_SIZE 1048576

void ubx_setup_basic(void);
void test_UBX(struct ubx_sbp_state state, const char *filename);

#endif /* CHECK_UBX_H */
