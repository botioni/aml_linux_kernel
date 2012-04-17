/*******************************************************************
 * 
 *  Copyright C 2010 by Amlogic, Inc. All Rights Reserved.
 *
 *  Description: 
 *
 *  Author: Herbert.hu 
 *  Created: 07/26 2011
 *
 *******************************************************************/

#ifndef __PCMENC_STREAM_H__
#define __PCMENC_STREAM_H__

/* initialize  stream FIFO 
 * return value: on success, zero is returned, on error, -1 is returned
 * */
extern int pcmenc_stream_init(void);

/* return space of  stream FIFO, unit:byte
 * */
extern int pcmenc_stream_space(void);

/* return content of  stream FIFO, unit:byte
 * */
extern int pcmenc_stream_content(void);

/* deinit  stream  FIFO 
 * return value: on success, zero is returned, on error, -1 is returned
 * */
extern int pcmenc_stream_deinit(void);

/* read  data out of FIFO, the minimum of the FIFO's content and size will be read, if the FIFO is empty, read will be failed 
 * return value: on success, the number of bytes read are returned, othewise, 0 is returned
 * */
extern int pcmenc_stream_read(unsigned char *buf, int size);

#endif

