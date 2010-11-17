#ifndef _SN7325_H_
#define _SN7325_H_

unsigned char get_configIO(unsigned char port);

int configIO(unsigned char port, unsigned char ioflag);

unsigned char getIObit_level(unsigned char port, unsigned char offset);

int setIO_level(unsigned char port, unsigned char iobits, unsigned char offset);

#endif