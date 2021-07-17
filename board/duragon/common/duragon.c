#include <common.h>
#include <dm.h>
#include <i2c.h>

#include "duragon.h"


#if !defined(CONFIG_TPL_BUILD)
#if 0
static int i2c_get_bit(struct udevice *dev, uint8_t reg, uint8_t bit)
{
	int ret;
	uint8_t value;
	ret = dm_i2c_read(dev, reg, &value, 1);
	if(ret < 0) {
		dev_err(dev, "%s: Reg: 0x%02x read failed, %d\n", dev->name, reg, ret);
		goto error;	
	}

	dev_info(dev, "%s: Reg: 0x%02x Value: 0x%02x\n", dev->name, reg, value);
	ret = (reg >> bit) & 0x01;

error:
	return (ret);
}
#endif
static int i2c_set_bit(struct udevice *dev, uint8_t reg, uint8_t bit, bool state)
{
	int ret;
	uint8_t value;
	ret = dm_i2c_read(dev, reg, &value, 1);
	if(ret < 0) {
		dev_err(dev, "%s: Reg: 0x%02x read failed, %d\n", dev->name, reg, ret);
		goto error;	
	}

	dev_info(dev, "%s: Reg: 0x%02x Value: 0x%02x\n", dev->name, reg, value);

	if(state) {
		value |= (1 << bit);
	}
	else {
		value &= ~(1 << bit);
	}

	ret = dm_i2c_write(dev, reg, &value, 1);
	if(ret < 0) {
		dev_err(dev, "%s: Reg: 0x%02x write failed, %d\n", dev->name, reg, ret);
	}

error:
	return (ret);
}

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
		debug("%s: chosen i2c bus failed\n", __func__);
		goto error;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, i2c_node, &i2c_bus);
	if (ret) {
		debug("%s: i2cbus get failed, %d\n", __func__, ret);
		goto error;
	}

	ret = dm_i2c_probe(i2c_bus, 0x48, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &lm75);
	if(ret) {
		dev_err(i2c_bus, "%s, lm75 probe failed, %d\n", i2c_bus->name, ret);
		goto error;
	}
	
	ret = dm_i2c_read(lm75, 0x00, data, 2);
	if(ret < 0) {
		dev_err(lm75, "%s: temperature read error, %d\n", lm75->name, ret);
		goto error;
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
	dev_info(lm75, "%s: RECEIVED: 0x%04x RAW: 0x%04x\n", lm75->name, lm75_data, temp_data);
	dev_info(lm75, "%s: Board Temperature: %d\n", lm75->name, temperature);

error:
	return temperature;
}

/**
 * dev	-	device instance after probed
 * pin	-	pin index
 * dir	-	Input/Output 1/0
 **/

static int pca9534a_set_direction(struct udevice *dev, int pin, int direction) 
{
	int ret;
	dev_info(dev, "%s: pin_%d direction -> %s\n", dev->name, pin, direction ? "OUTPUT": "INPUT");
	if(!dev) {
		ret = ENXIO;
		goto error;
	}

	ret = dm_i2c_reg_read(dev, 0x03);
	if(ret < 0) {
		dev_err(dev, "%s: read failed!, %d\n", dev->name, ret);
		goto error;
	}
	
	dev_info(dev, "%s: 0x03 -> 0x%02x\n", dev->name, ret);
	if(!direction) {
		ret |= (1 << pin);
	}
	else {
		ret &= ~(1 << pin);
	}
	dev_info(dev, "%s: 0x03 -> 0x%02x\n", dev->name, ret);

	ret = dm_i2c_reg_write(dev, 0x03, ret);
	if(ret < 0) {
		dev_err(dev, "%s: write failed, %d\n", dev->name, ret);
	}

error:
	return ret;
}

static int pca9534a_set_value(struct  udevice *dev, int pin, int value)
{
	int ret;
	dev_info(dev, "%s: pin_%d value -> %d\n",dev->name, pin, value);
	if(!dev) {
		ret = ENXIO;
		goto error;
	}

	ret = dm_i2c_reg_read(dev, 0x01);
	if(ret < 0) {
		dev_err(dev, "%s: read failed!, %d\n", dev->name, ret);
		goto error;
	}
	
	dev_info(dev, "%s: 0x01 -> 0x%02x\n", dev->name, ret);
	if(value) {
		ret |= (1 << pin);
	}
	else {
		ret &= ~(1 << pin);
	}

	dev_info(dev, "%s: 0x01 -> 0x%02x\n", dev->name, ret);
	ret = dm_i2c_reg_write(dev, 0x01, ret);
	if(ret < 0) {
		dev_err(dev, "%s: write failed, %d\n", dev->name, ret);
	}

error:
	return (ret);
};

void setup_fpdlink_iii(void)
{
	struct udevice *i2c_bus;
	struct udevice *edp_to_lvds;
	struct udevice *serializer;
	struct udevice *deserializer;
	struct udevice *io_expander;
	ofnode i2c_node;
	int ret;
	unsigned char data[8];
	bool forward_channel_crc = false;
	bool backward_channel_crc = false;
	unsigned short crc_errors = 0;
	uint8_t id[7] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0' };	


	i2c_node = ofnode_get_chosen_node("duragon,i2c");
	if(!ofnode_valid(i2c_node)) {
		debug("%s: chosen i2c bus failed\n", __func__);
		return;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_I2C, i2c_node, &i2c_bus);
	if (ret) {
		debug("%s: i2cbus get failed, %d\n", __func__, ret);
		return;
	}

	ret = dm_i2c_probe(i2c_bus, 0x60, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &edp_to_lvds);
	if(ret) {
		dev_err(i2c_bus, "%s: ptn3460 probe failed, %d\n", i2c_bus->name, ret);
		return;
	}

	dev_info(serializer, "%s: eDP -> LVDS\n", i2c_bus->name);	
	/**
	 * LVDS interface control 1
	 * Address: 0x81
	 * bit0_1: Clocck output for dual LVDS mode
	 * bit2:    Data enable polarity
	 * bit3:  Dual LVDS mode
	 * bit4_5: Color depth
	 * 			00	-	VESA 24bpp
	 * 			01	-	JEIDA 24bpp
	 * 			10	-	VESA and JEIDA18bpp
	 * 			11	-	reserved
	 * bit7_6:	Reserved
	 **/
	ret = dm_i2c_reg_write(edp_to_lvds, 0x81, 0x10);
	if(ret < 0) {
		dev_err(edp_to_lvds, "%s: Color depth and data packing format failed, %d\n", edp_to_lvds->name, ret);
	}

	/**
	 * Backlight control
	 * Address: 0x95
	 * bit0: enable/disable 0/1
	 * bit1_7: reserved
	 * We are not using backlight features from ptn3650
	 **/
	ret = dm_i2c_reg_write(edp_to_lvds, 0x95, 0x01);
	if(ret < 0) {
		dev_err(edp_to_lvds, "%s: backlight control failed, %d\n", edp_to_lvds->name, ret);
	}

	ret = dm_i2c_probe(i2c_bus, 0x1b, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &serializer);
	if(ret) {
		dev_err(i2c_bus, "%s: serializer probe failed, %d\n", i2c_bus->name, ret);
		return;
	}

	dev_info(serializer, "%s: LVDS -> FPD Link III\n", i2c_bus->name);
	ret = dm_i2c_read(serializer, 0xF0, id, 6);
	if(ret < 0) {
		dev_err(serializer, "%s: ID read failed, %d\n", serializer->name, ret);
		return;
	}

	dev_info(serializer, "%s: %s\n", serializer->name, id);

	/**
	 * General Status
	 * Address: 0x0C
	 * bit0: Link Detect 0/1 no/yes
	 * bit1: Forward Channel CRC Error during BIST 0/1 no/yes
	 * bit2: PCLK detected 0/1 no/yes
	 * bit3: Back Channel CRC Error during BIST 0/1 no/yes	 * 
	 **/	
	ret = dm_i2c_read(serializer, 0x0C, data, 1);
	if(ret < 0) {
		dev_err(serializer, "%s: status read failed, %d\n", serializer->name, ret);
		return;
	}

	if(data[0] & (1 << 0)) {
		dev_info(serializer, "%s: Link detected\n", serializer->name);
	}
	else {
		dev_alert(serializer, "%s: Link not detected\n", serializer->name);
	}

	if(data[0] & (1 << 2)) {
		dev_info(serializer, "%s: PCLK detected\n", serializer->name);
	}
	else {
		dev_alert(serializer, "%s: PCLK not detected\n", serializer->name);
	}

	if(data[0] & (1 << 1)) {
		dev_alert(serializer, "%s: Forward Channel CRC Error detected\n", serializer->name);
		forward_channel_crc = true;
	}

	if(data[0] & (1 << 3)) {
		dev_alert(serializer, "%s: Back Channel CRC Error detected\n", serializer->name);
		backward_channel_crc = true;
	}

	if(forward_channel_crc || backward_channel_crc) {
		/* Read and Reset */
		ret = dm_i2c_read(serializer, 0x0A, data, 2);
		if(ret < 0) {
			dev_err(serializer, "%s: CRC Errors read failed, %d\n", serializer->name, ret);
		}
		else {
			crc_errors = (data[1] << 8) | data[0];
			dev_alert(serializer, "%s: CRC Errors: 0x%04x\n", serializer->name, crc_errors);
		}

		ret = dm_i2c_reg_write(serializer, 0x04, 0xA0);
		if(ret < 0) {
			dev_err(serializer, "%s: CRC reset failed, %d\n", serializer->name, ret);
		}
		
		ret = dm_i2c_reg_write(serializer, 0x04, 0x80);
		if(ret < 0) {
			dev_err(serializer, "%s: CRC reset failed, %d\n", serializer->name, ret);
		}
		
	}

	/**
	 * Deserializer ID
	 * Address: 0x06
	 * bit1_7: 7bit
	 **/
	ret = dm_i2c_read(serializer, 0x06, data, 1);
	if(ret < 0) {
		dev_err(serializer, "%s: Remote ID read failed, %d\n", serializer->name, ret);
		return;
	}
	else {
		dev_info(serializer, "%s: Remote ID: 0x%02x\n", serializer->name, (data[0] >> 1));
	}

	if((data[0] >> 1) == 0x3b) {
		dev_info(serializer, "%s: DeSerializer found\n", serializer->name);
	}
	else {
		dev_info(serializer, "%s: Unknown deserializer\n", serializer->name);
		return;
	}

	/**
	 * ICR
	 * Address: 0xc6
	 * bit5: Enable 0/1 yes/no
	 **/
	ret = i2c_set_bit(serializer, 0xc6, 5, true);
	if(ret < 0) {
		dev_err(serializer, "%s: interrupt enable failed, %d", serializer->name, ret);
		return;
	}
	
	/**
	 * GPIO1 Configuration
	 * Address: 0x0e
	 * bit0: Enable GPIO
	 * bit1: direction 0/1 out/in
	 * bit2: remote control 0/1 enable/disable
	 * bit3: Outputvalue 0/1 low/high
	 * bit4_7: Revision ID
	 **/
	ret = i2c_set_bit(serializer, 0x0e, 2, true);
	if(ret < 0) {
		dev_err(serializer, "%s: remote gpio control failed, %d", serializer->name, ret);
	}
	ret = i2c_set_bit(serializer, 0x0e, 0, true);
	if(ret < 0) {
		dev_err(serializer, "%s: enable gpio operation failed, %d", serializer->name, ret);
	}

	/**
	 * ISR
	 * address: 0xc7
	 * bit5: Interrupt on deserializer int
	 * bit0: Global Interrupt
	 * Read this register activate the interrupt
	 **/
	ret = dm_i2c_read(serializer, 0xc7, data, 1); /* Dummy read */
	ret = dm_i2c_read(serializer, 0xc7, data, 1);
	if(ret < 0) {
		dev_err(serializer, "%s: interrupt read failed, %d\n", serializer->name, ret);
		return;
	}
	dev_info(serializer, "%s: Remote: %s\n", serializer->name, ((data[0] & (1 << 5)) ? "INTERRUPT" : "NO INTERRUPT"));
	dev_info(serializer, "%s: Global: %s\n", serializer->name, ((data[0] & (1 << 0)) ? "INTERRUPT" : "NO INTERRUPT"));

	/**
	 * I2C Control
	 * Adress: 0x17
	 * bit0_3: pulse glitch
	 * bit4_6: sda hold time
	 * bit7: PassAll
	 **/
	ret = i2c_set_bit(serializer, 0x17, 7, true);
	if(ret < 0) {
		dev_err(serializer, "%s: PASS ALL failed, %d\n", serializer->name, ret);
		return;
	}

	dev_info(serializer, "%s: FPD Link III -> LVDS\n", i2c_bus->name);
	ret = dm_i2c_probe(i2c_bus, 0x3b, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &deserializer);
	if(ret < 0) {
		dev_err(i2c_bus, "%s: DeSerializer probe failed, %d\n", i2c_bus->name, ret);
		return;
	}
	/**
	 * I2C Control
	 * Adress: 0x17
	 * bit0_3: Filter
	 * bit4_6: sda hold time
	 * bit7: PassAll
	 **/
#if 0	/* No need when deserializer side no master */
	ret = i2c_set_bit(deserializer, 0x05, 7, true);
	if(ret < 0) {
		dev_err(deserializer, "%s: PASS ALL failed, %d\n", deserializer->name, ret);
		return;
	}
#endif

	/**
	 * Remote ID
	 * Address: 0x07
	 * bit1_7: 7bit id
	 **/

	ret = dm_i2c_read(deserializer, 0x07, data, 1);
	if(ret < 0) {
		dev_err(deserializer, "%s: Remote ID read failed, %d\n", deserializer->name, ret);
		return;
	}
	dev_info(deserializer, "%s: Remote ID: 0x%02x\n", deserializer->name, (data[0] >> 1));
	
	/**
	 * GPIO1 Configuration
	 * Address: 0x1E
	 * bit0: Enable GPIO
	 * bit1: direction 0/1 out/in
	 * bit2: remote control 0/1 enable/disable
	 * bit3: Outputvalue 0/1 low/high
	 * bit4_7: Revision ID
	 **/
	ret = i2c_set_bit(deserializer, 0x1e, 1, true);
	if(ret < 0) {
		dev_err(deserializer, "%s: setting gpio input failed, %d", deserializer->name, ret);
	}
	ret = i2c_set_bit(deserializer, 0x1e, 0, true);
	if(ret < 0) {
		dev_err(deserializer, "%s: enable gpio operation failed, %d", deserializer->name, ret);
	}

	dev_info(deserializer, "%s: Panel IO\n", i2c_bus->name);
	ret = dm_i2c_probe(i2c_bus, 0x38, DM_I2C_CHIP_RD_ADDRESS | DM_I2C_CHIP_WR_ADDRESS, &io_expander);
	if(ret < 0) {
		dev_err(i2c_bus, "%s: IOExpander probe failed, %d\n", i2c_bus->name, ret);
		return;
	}

	/* IO Expander set display enable signal low */
	ret = pca9534a_set_value(io_expander, 0, 0);
	if(ret < 0) {
		dev_err(io_expander, "%s: set failed, %d\n", io_expander->name, ret);
		return;
	}
	
	ret = pca9534a_set_direction(io_expander, 0, 1);
	if(ret < 0) {
		dev_err(io_expander, "%s: direction failed, %d\n", io_expander->name, ret);
		return;
	}

	ret = dm_i2c_reg_read(io_expander, 0x00);
	if(ret < 0) {
		dev_err(io_expander, "%s: Input port read failed, %d\n", io_expander->name, ret);
		return;
	}
	dev_info(io_expander, "%s: 0x00 -> 0x%02x\n", io_expander->name, ret);
	ret = dm_i2c_reg_read(io_expander, 0x00);
	if(ret < 0) {
		dev_err(io_expander, "%s: Input port read failed, %d\n", io_expander->name, ret);
		return;
	}
	dev_info(io_expander, "%s: 0x00 -> 0x%02x\n", io_expander->name, ret);
}
#endif

#if defined(CONFIG_SPL_BUILD)
void spl_board_init(void)
{
	debug("%s\n", __func__);
}
#endif