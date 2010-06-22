#ifndef MESON_PINMUX_H
#define MESON_PINMUX_H
/*
Linux Pinmux.h
*/

int clear_mio_mux(unsigned mux_index, unsigned mux_mask);
int set_mio_mux(unsigned mux_index, unsigned mux_mask);
#endif

