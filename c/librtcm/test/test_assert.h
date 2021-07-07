/*
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef TEST_ASSERT_H
#define TEST_ASSERT_H

#ifdef NDEBUG
#define _NDEBUG_PREV NDEBUG
#undef NDEBUG
#include <assert.h>
#define NDEBUG _NDEBUG_PREV
#undef _NDEBUG_PREV
#else
#include <assert.h>
#endif

#endif
