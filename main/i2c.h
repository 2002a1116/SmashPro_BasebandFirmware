uint8_t i2c_write_data(i2c_master_dev_handle_t* dev,uint8_t addr,uint8_t* buf,int32_t length);
uint8_t i2c_read_reg(i2c_master_dev_handle_t* dev,uint8_t addr,uint8_t* buf,int32_t length);
void i2c_init(void);