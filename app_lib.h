#ifndef _APP_LIB_
#define _APP_LIB_

signed char i2c_write(unsigned char slave_addr,
					  unsigned char reg_addr,
                      unsigned char length,
                      unsigned char const *data);
					  
signed char i2c_read(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char *data);
					 
int delay_ms(unsigned char delay);

int get_ms(unsigned long *count);
					 
#endif  /* _APP_LIB_ */
					 
					 