#ifndef COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_
#define COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_

#include "driver/rmt.h"

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

private:
	struct dshot_packet_t
	{
		uint16_t payload;
		bool telemetry;
	};

	void setData(uint16_t data);
	static uint8_t checksum(uint16_t data);
	esp_err_t writeData(uint16_t data, bool wait);
	esp_err_t writePacket(dshot_packet_t packet, bool wait);
	esp_err_t repeatPacket(dshot_packet_t packet, int n);
	esp_err_t repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);


	rmt_item32_t _dshotCmd[17];
	rmt_channel_t _rmtChannel;
};

#endif /* COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_ */
