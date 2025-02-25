/****************************************************************************
 * libs/libc/stream/lib_memoutstream.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memoutstream_puts
 ****************************************************************************/

static ssize_t memoutstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buf, size_t len)
{
  FAR struct lib_memoutstream_s *stream =
                                (FAR struct lib_memoutstream_s *)self;
  size_t ncopy;

  DEBUGASSERT(self);

  /* If this will not overrun the buffer, then write the character to the
   * buffer.  Note that buflen was pre-decremented when the stream was
   * created so it is okay to write past the end of the buflen by one.
   */

  ncopy = stream->buflen - self->nput >= len ?
          len : stream->buflen - self->nput;
  if (ncopy > 0)
    {
      memcpy(stream->buffer + self->nput, buf, ncopy);
      self->nput += ncopy;
      stream->buffer[self->nput] = '\0';
    }

  return ncopy;
}

/****************************************************************************
 * Name: memoutstream_putc
 ****************************************************************************/

static void memoutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  char tmp = ch;
  memoutstream_puts(self, &tmp, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_memoutstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input Parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *                  lib_memoutstream_s to be initialized.
 *   bufstart     - Address of the beginning of the fixed-size memory buffer
 *   buflen       - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (outstream initialized).
 *
 ****************************************************************************/

void lib_memoutstream(FAR struct lib_memoutstream_s *outstream,
                      FAR char *bufstart, size_t buflen)
{
  outstream->common.putc  = memoutstream_putc;
  outstream->common.puts  = memoutstream_puts;
  outstream->common.flush = lib_noflush;
  outstream->common.nput  = 0;          /* Will be buffer index */
  outstream->buffer       = bufstart;   /* Start of buffer */
  outstream->buflen       = buflen - 1; /* Save space for null terminator */
  outstream->buffer[0]    = '\0';       /* Start with an empty string */
}
