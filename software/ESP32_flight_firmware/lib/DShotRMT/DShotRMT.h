#ifndef COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_
#define COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_

#include "driver/rmt.h"

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

// from https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
enum dshot_cmd_t
{
	DIGITAL_CMD_MOTOR_STOP, 		      // Currently not implemented
	DIGITAL_CMD_BEEP1, 			      // Wait at least length of beep (260ms) before next command
	DIGITAL_CMD_BEEP2, 			      // Wait at least length of beep (260ms) before next command
	DIGITAL_CMD_BEEP3, 			      // Wait at least length of beep (280ms) before next command
	DIGITAL_CMD_BEEP4, 			      // Wait at least length of beep (280ms) before next command
	DIGITAL_CMD_BEEP5, 			      // Wait at least length of beep (1020ms) before next command
	DIGITAL_CMD_ESC_INFO,  		      // Wait at least 12ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_1, 	      // Need 6x, no wait required
	DIGITAL_CMD_SPIN_DIRECTION_2, 	      // Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_OFF, 		      // Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_ON,  		      // Need 6x, no wait required
	DIGITAL_CMD_SETTINGS_REQUEST,  	      // Currently not implemented
	DIGITAL_CMD_SAVE_SETTINGS,  		      // Need 6x, wait at least 35ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_NORMAL = 20, 	      // Need 6x, no wait required
	DIGITAL_CMD_SPIN_DIRECTION_REVERSED, 	      // Need 6x, no wait required
	DIGITAL_CMD_LED0_ON, 			      // No wait required
	DIGITAL_CMD_LED1_ON, 			      // No wait required
	DIGITAL_CMD_LED2_ON, 			      // No wait required
	DIGITAL_CMD_LED3_ON, 			      // No wait required
	DIGITAL_CMD_LED0_OFF, 		      // No wait required
	DIGITAL_CMD_LED1_OFF, 		      // No wait required
	DIGITAL_CMD_LED2_OFF, 		      // No wait required
	DIGITAL_CMD_LED3_OFF, 		      // No wait required
};


struct dshot_packet_t
	{
		uint16_t payload;
		bool telemetry;
	};

class DShotRMT
{
public:
	DShotRMT();
	~DShotRMT();

	esp_err_t install(gpio_num_t gpio, rmt_channel_t rmtChannel);
	esp_err_t uninstall();

	esp_err_t init(bool wait = true);
	esp_err_t sendThrottle(uint16_t throttle);
	esp_err_t sendPacket(uint16_t packet);
	esp_err_t setReversed(bool reversed);
	esp_err_t beep();
	
	esp_err_t writeData(uint16_t data, bool wait);
	esp_err_t writePacket(dshot_packet_t packet, bool wait);
	
	rmt_channel_t _rmtChannel;

private:

	void setData(uint16_t data);
	static uint8_t checksum(uint16_t data);
	esp_err_t repeatPacket(dshot_packet_t packet, int n);
	esp_err_t repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);

	rmt_item32_t _dshotCmd[17];
};

#endif /* COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_ */
