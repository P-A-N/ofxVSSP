/*!
  \file
  \brief functions to hanle ring buffer

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg3d_ring_buffer.h"


void urg3d_ring_initialize(urg3d_ring_buffer_t *ring
                     , char *buffer
                     , const int shift_length)
{
    ring->buffer = buffer;
    ring->buffer_size = 1 << shift_length;
    urg3d_ring_clear(ring);
}


void urg3d_ring_clear(urg3d_ring_buffer_t *ring)
{
    ring->first = 0;
    ring->last = 0;
}


int urg3d_ring_size(const urg3d_ring_buffer_t *ring)
{
    int first = ring->first;
    int last = ring->last;

    return (last >= first) ? last - first : ring->buffer_size - (first - last);
}


int urg3d_ring_capacity(const urg3d_ring_buffer_t *ring)
{
    return ring->buffer_size - 1;
}


static void byte_move(char *dest
                      , const char *src
                      , int n)
{
    const char *last_p = dest + n;
    while (dest < last_p) {
        *dest++ = *src++;
    }
}


int urg3d_ring_write(urg3d_ring_buffer_t *ring
               , const char *data
               , int size)
{
    int free_size = urg3d_ring_capacity(ring) - urg3d_ring_size(ring);
    int push_size = (size > free_size) ? free_size : size;

    // storing data
    if (ring->first <= ring->last) {
        // set from last to end point (length : buffer_size)
        int left_size = 0;
        int to_end = ring->buffer_size - ring->last;
        int move_size = (to_end > push_size) ? push_size : to_end;

        byte_move(&ring->buffer[ring->last], data, move_size);
        ring->last += move_size;
        ring->last &= (ring->buffer_size -1);

        left_size = push_size - move_size;
        if (left_size > 0) {
            // set from 0 to before first
            byte_move(ring->buffer, &data[move_size], left_size);
            ring->last = left_size;
        }
    } else {
        // set from last to before first
        byte_move(&ring->buffer[ring->last], data, size);
        ring->last += push_size;
    }
    return push_size;
}


int urg3d_ring_read(urg3d_ring_buffer_t *ring
              , char *buffer
              , int size)
{
    // receive data
    int now_size = urg3d_ring_size(ring);
    int pop_size = (size > now_size) ? now_size : size;

    if (ring->first <= ring->last) {
        byte_move(buffer, &ring->buffer[ring->first], pop_size);
        ring->first += pop_size;

    } else {
        // set from first to end point (length : buffer_size)
        int left_size = 0;
        int to_end = ring->buffer_size - ring->first;
        int move_size = (to_end > pop_size) ? pop_size : to_end;
        byte_move(buffer, &ring->buffer[ring->first], move_size);

        ring->first += move_size;
        ring->first &= (ring->buffer_size -1);

        left_size = pop_size - move_size;
        if (left_size > 0) {
            // set from 0 to before last
            byte_move(&buffer[move_size], ring->buffer, left_size);

            ring->first = left_size;
        }
    }
    return pop_size;
}
