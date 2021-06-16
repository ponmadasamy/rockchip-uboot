#include <common.h>
#include <dm.h>
#include <i2c.h>

int board_temperature(void)
{
	struct udevice *i2c_bus;
	struct udevice *lm75;
	ofnode i2c_node;
	int ret;
	unsigned char data[8];
	unsigned short lm75_data = 0;
	unsigned short temp_data = 0;
	int temperature = 0.0;

	i2c_node = ofnode_get_chosen_node("duragon,i2c");
	if(!ofnode_valid(i2c_node)) {
		debug("%s: Invalid i2c3 node\n", __func__);
		goto out;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, i2c_node, &i2c_bus);
	if (ret) {
		debug("%s: Cannot find i2cbus 3, %d\n", __func__, ret);
		goto out;
	}

	ret = dm_i2c_probe(i2c_bus, 0x48, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &lm75);
	if(ret) {
		debug("%s, Cannot find lm75, %d\n", __func__, ret);
	}
	else {
		ret = dm_i2c_read(lm75, 0x00, data, 2);
		if(ret < 0) {
			debug("Board Temperature: read error, %d\n", ret);
		}
		lm75_data = 0;
		lm75_data |= data[0] << 8;
		lm75_data |= data[1];
		temp_data = lm75_data >> 4;
		/* 
		if(temp_data & (1 < 10)) {
			debug("Negative Temperature\n");
			temp_data = (~temp_data + 1);
		}
		else {}
		temperature = temp_data * 0.125;
		*/
		temperature = temp_data;
		debug("RECEIVED: 0x%04x RAW: 0x%04x\n", lm75_data, temp_data);
		debug("Board Temperature: %d\n", temperature);
	}

out:
	return temperature;
}

/**
 * dev	-	device instance after probed
 * pin	-	pin index
 * dir	-	Input/Output 1/0
 **/

static int set_direction(struct udevice *dev, int pin, int direction) 
{
	int ret;
	debug("set_direction: %d -> %s\n", pin, direction ? "OUTPUT": "INPUT");
	if(dev) {
		ret = dm_i2c_reg_read(dev, 0x03);
		if(ret < 0) {
			debug("IO Expender read failed!, %d\n", ret);
		}
		else {
			debug("Vaule from register: %d\n", ret);
			if(!direction) {
				ret |= (1 << pin);
			}
			else {
				ret &= ~(1 << pin);
			}
			debug("Vaule to register: %d\n", ret);
			ret = dm_i2c_reg_write(dev, 0x03, ret);
			if(ret < 0) {
				debug("IO Expander write failed, %d\n", ret);
			}
		}
	}

	return ret;
}

static int set_value(struct  udevice *dev, int pin, int value)
{
	int ret;
	debug("set_value: %d -> %d\n", pin, value);
	if(dev) {
		ret = dm_i2c_reg_read(dev, 0x01);
		if(ret < 0) {
			debug("IO Expender read failed!, %d\n", ret);
		}
		else {
			debug("Vaule from register: %d\n", ret);
			if(value) {
				ret |= (1 << pin);
			}
			else {
				ret &= ~(1 << pin);
			}
			debug("Vaule to register: %d\n", ret);
			ret = dm_i2c_reg_write(dev, 0x01, ret);
			if(ret < 0) {
				debug("IO Expander write failed, %d\n", ret);
			}
		}
	}

	return (ret);
};

void setup_fpdlink_iii(void)
{
	struct udevice *i2c_bus;
	struct udevice *serializer;
	struct udevice *io_expander;
	ofnode i2c_node;
	int ret;
	unsigned char data[8];
	bool forward_channel_crc = false;
	bool backward_channel_crc = false;
	unsigned short crc_errors = 0;


	i2c_node = ofnode_get_chosen_node("duragon,i2c");
	if(!ofnode_valid(i2c_node)) {
		debug("%s: Invalid i2c3 node\n", __func__);
		return;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, i2c_node, &i2c_bus);
	if (ret) {
		debug("%s: Cannot find i2cbus 3, %d\n", __func__, ret);
		return;
	}

	ret = dm_i2c_probe(i2c_bus, 0x1b, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &serializer);
	if(ret) {
		debug("%s, Cannot find serializer, %d\n", __func__, ret);
	}
	else {
		/* Read Status */
		ret = dm_i2c_read(serializer, 0x0C, data, 1);
		if(ret < 0) {
			debug("Serializer status read failed, %d\n", ret);
		}
		else {
			if(data[0] & (0x01)) {
				debug("Serializer cable link detected\n");
			}
			else {
				debug("Serializer cable link not detected\n");
			}

			if(data[0] & (1 << 2)) {
				debug("Serializer valid PCLCK detected\n");
			}
			else {
				debug("Serializer no PCLK\n");
			}

			if(data[0] & (1 << 1)) {
				debug("Serializer Forward Channel CRC Error detected\n");
				forward_channel_crc = true;
			}

			if(data[0] & (1 << 3)) {
				debug("Serializer Back Channel CRC Error detected\n");
				backward_channel_crc = true;
			}

			if(forward_channel_crc || backward_channel_crc) {
				/* Read and Reset */
				ret = dm_i2c_read(serializer, 0x0A, data, 2);
				if(ret < 0) {
					debug("Serializer CRC Errors read failed, %d\n", ret);
				}
				else {
					crc_errors = (data[1] << 8) | data[0];
					debug("Serializer CRC Errors: 0x%04x\n", crc_errors);
				}

				ret = dm_i2c_reg_write(serializer, 0x04, 0xA0);
				if(ret < 0) {
					debug("Serializer CRC reset failed, %d\n", ret);
				}
				
				ret = dm_i2c_reg_write(serializer, 0x04, 0x80);
				if(ret < 0) {
					debug("Serializer CRC reset failed, %d\n", ret);
				}
				
			}

			ret = dm_i2c_read(serializer, 0x06, data, 1);
			if(ret < 0) {
				debug("Serializer DeSerializer ID read failed, %d\n", ret);
			}
			else {
				debug("Serializer DeSerializer ID: 0x%02x\n", (data[0] >> 1));

				if((data[0] >> 1) == 0x3b) {
					debug("Serializer matching DeSerializer found\n");
					/* Pass All */
					ret = dm_i2c_reg_write(serializer, 0x17, 0x9E);
					if(ret < 0) {
						debug("Serializer PASS ALL failed, %d\n", ret);
					}

					ret = dm_i2c_probe(i2c_bus, 0x38, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &io_expander);
					if(ret < 0) {
						debug("IOExpander probe failed, %d\n", ret);
					}

					/* IO Expander set display run signal low */
					set_value(io_expander, 0, 0);
					set_direction(io_expander, 0, 1);
				}
			}
		}

	}
}