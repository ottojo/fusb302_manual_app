#include <stdbool.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/usb_c/fusb302b.h>
#include <zephyr/drivers/usb_c/usbc_pd.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


const struct device *const comp1 = DEVICE_DT_GET(DT_NODELABEL(comp1));
const struct device *const comp2 = DEVICE_DT_GET(DT_NODELABEL(comp2));

#define I2C_NODE DT_NODELABEL(i2c_internal)

#define ERROR_CHECK(res)                                                                                               \
	if (res != 0) {                                                                                                    \
		LOG_ERR("Error in i2c read: %d", res);                                                                         \
		log_panic();                                                                                                   \
		return 1;                                                                                                      \
	}

static const uint8_t REG_SWITCHES0 = 0x02;
static const uint8_t REG_MEASURE = 0x04;
static const uint8_t REG_SWITCHES1 = 0x03;
static const uint8_t REG_CONTROL0 = 0x06;
static const uint8_t REG_CONTROL1 = 0x07;
static const uint8_t REG_CONTROL3 = 0x09;
static const uint8_t REG_POWER = 0x0b;
static const uint8_t REG_RESET = 0x0c;
static const uint8_t REG_STATUS0 = 0x40;
static const uint8_t REG_STATUS1 = 0x41;
static const uint8_t REG_FIFO = 0x43;

static const uint8_t SOP1 = 0x12;
static const uint8_t SOP2 = 0x13;
static const uint8_t PACKSYM = 0x80;
static const uint8_t JAM_CRC = 0xFF;
static const uint8_t EOP = 0x14;
static const uint8_t TXOFF = 0xFE;
static const uint8_t TXON = 0xA1;

#define FUSB302_RX_BUFFER_SIZE 80


int read_message(const struct i2c_dt_spec *spec, struct pd_msg *buf) {
	uint8_t sop_token;
	int res = i2c_reg_read_byte_dt(spec, REG_FIFO, &sop_token);
	if (res != 0) { return -EIO; }

	/* First byte determines package type */
	switch (sop_token >> 5) {
		case 0b111:
			buf->type = PD_PACKET_SOP;
			LOG_DBG("Packet type SOP");
			break;
		case 0b110:
			buf->type = PD_PACKET_SOP_PRIME;
			LOG_DBG("Packet type SOP_P");
			break;
		case 0b101:
			buf->type = PD_PACKET_PRIME_PRIME;
			LOG_DBG("Packet type SOP_P_P");
			break;
		case 0b100:
			buf->type = PD_PACKET_DEBUG_PRIME;
			LOG_DBG("Packet type SOP_D_P");
			break;
		case 0b011:
			buf->type = PD_PACKET_DEBUG_PRIME_PRIME;
			LOG_DBG("Packet type SOP_D_P_P");
			break;
		default:
			LOG_ERR("Read unknown start-token from RxFIFO: %#04x", sop_token);
			return -EIO;
	}
	uint8_t header[2];

	res = i2c_burst_read_dt(spec, REG_FIFO, header, 2);
	if (res != 0) {
		LOG_ERR("Error while reading from fifo");
		return -EIO;
	}
	buf->header.raw_value = header[0] | (header[1] << 8);
	LOG_HEXDUMP_DBG(header, sizeof(header), "RX header:");

	buf->len = PD_CONVERT_PD_HEADER_COUNT_TO_BYTES(buf->header.number_of_data_objects);
	__ASSERT(buf->len <= sizeof(buf->data), "Packet size of %d is larger than buffer of size %d", buf->len,
			 sizeof(buf->data));
	__ASSERT(buf->len <= (FUSB302_RX_BUFFER_SIZE - 3), "Packet size of %d is larger than FUSB302B RxFIFO", buf->len);
	LOG_DBG("Reading %d data bytes", buf->len);
	if (buf->len > 0) {
		res = i2c_burst_read_dt(spec, REG_FIFO, buf->data, buf->len);
		if (res != 0) { return -EIO; }
		LOG_HEXDUMP_DBG(buf->data, buf->len, "RX data:");
	}

	/* Read CRC */
	uint8_t crc[4];

	res = i2c_burst_read_dt(spec, REG_FIFO, crc, 4);
	if (res != 0) { return -EIO; }

	return buf->len + 2 /* header */;
}

void wait_until_message(const struct i2c_dt_spec *spec) {
	LOG_INF("Waiting for message...");
	while (true) {
		uint8_t status1 = 0;
		int res = i2c_reg_read_byte_dt(spec, REG_STATUS1, &status1);
		if (res != 0) { LOG_ERR("Error!"); }

		if ((status1 & 0b00100000) == 0) {
			LOG_INF("RX buffer non-empty!");
			return;
		}
		k_usleep(500);
	}
}

void handle_pdo(int i, uint32_t pdo_value) {
	union pd_fixed_supply_pdo_source pdo;
	pdo.raw_value = pdo_value;
	if (pdo.type != PDO_FIXED) {
		LOG_WRN("type of PDO %d is not fixed!", i);
		return;
	}
	LOG_INF("PDO %d: %dmV", i, pdo.voltage * 50);
}

int main() {
	if (!device_is_ready(comp1)) {
		LOG_ERR("COMP1 not init");
		while (true) { k_msleep(100); }
	}

	if (!device_is_ready(comp2)) {
		LOG_ERR("COMP2 not init");
		while (true) { k_msleep(100); }
	}

	const struct device *const i2c_bus = DEVICE_DT_GET(I2C_NODE);
	if (!device_is_ready(i2c_bus)) {
		LOG_ERR("i2c not init");
		while (true) { k_msleep(100); }
	}

	LOG_INF("Startup complete");

	struct i2c_dt_spec i2c = {.bus = i2c_bus, .addr = 0x22};

	int res;

	LOG_INF("Resetting FUSB302B");
	res = i2c_reg_write_byte_dt(&i2c, REG_RESET, 0x01);
	ERROR_CHECK(res);

	LOG_INF("Powering on chip");
	res = i2c_reg_write_byte_dt(&i2c, REG_POWER, 0x0f);
	ERROR_CHECK(res);

	LOG_INF("Unmasking interrupts");
	// Note: this also changes pullup current, but pullup is not relevant for sink.
	res = i2c_reg_write_byte_dt(&i2c, REG_CONTROL0, 0x00);
	ERROR_CHECK(res);


	LOG_INF("Enabling packet retries");
	res = i2c_reg_write_byte_dt(&i2c, REG_CONTROL3, 0x07);
	ERROR_CHECK(res);


	LOG_INF("Reading device id");
	uint8_t deviceid = 0;
	res = i2c_reg_read_byte_dt(&i2c, 0x01, &deviceid);
	ERROR_CHECK(res);

	LOG_INF("Read device id 0x%02x", deviceid);
	if ((deviceid & 0xF0) != 0x90) {
		LOG_ERR("Unexpected device id");
		return 1;
	}

	// Wait for VBUS
	LOG_INF("Waiting for VBUS");

	LOG_INF("Connecting measure block to vbus");

	res = i2c_reg_write_byte_dt(&i2c, REG_SWITCHES0, 0b11);// disconnect MEAS_CC
	ERROR_CHECK(res);
	res = i2c_reg_write_byte_dt(&i2c, REG_MEASURE, 0b01001010);// MEAS_VBUS=1, MDAC=10 (10+1)*420mV=4.62V
	ERROR_CHECK(res);

	{
		uint8_t status0 = 0;
		do {
			k_usleep(350);// Allow measurement to settle
			res = i2c_reg_read_byte_dt(&i2c, REG_STATUS0, &status0);
			ERROR_CHECK(res);
		} while ((status0 & 0b00100000) == 0);
	}
	LOG_INF("VBUS Connected!");

	LOG_INF("Disconnecting measure block from vbus");
	res = i2c_reg_write_byte_dt(&i2c, REG_MEASURE, 0b00110001);// Revert measure to default (2.1V)
	ERROR_CHECK(res);

	// Measure CC
	uint8_t cc = 0;

	while (cc == 0) {
		LOG_INF("Connect measure to CC1");
		res = i2c_reg_write_byte_dt(&i2c, REG_SWITCHES0, 0b0111);
		ERROR_CHECK(res);

		k_usleep(250);
		uint8_t status0 = 0;
		res = i2c_reg_read_byte_dt(&i2c, REG_STATUS0, &status0);
		ERROR_CHECK(res);
		uint8_t cc1 = status0 & 0b11;
		LOG_INF("Measured %d", cc1);

		LOG_INF("Connect measure to CC2");
		res = i2c_reg_write_byte_dt(&i2c, REG_SWITCHES0, 0b1011);
		ERROR_CHECK(res);

		k_usleep(250);
		res = i2c_reg_read_byte_dt(&i2c, REG_STATUS0, &status0);
		ERROR_CHECK(res);
		uint8_t cc2 = status0 & 0b11;
		LOG_INF("Measured %d", cc2);

		if (cc2 > cc1) {
			cc = 2;
		} else if (cc1 > cc2) {
			cc = 1;
		} else {
			LOG_ERR("Equal CC values.");
			k_msleep(50);
		}
	}

	LOG_INF("Selected CC %d", cc);

	// Select CC

	LOG_INF("Enabling TX, AUTO_CRC for CC %d", cc);
	res = i2c_reg_write_byte_dt(&i2c, REG_SWITCHES1, 0b00100100 | cc);
	ERROR_CHECK(res);

	LOG_INF("Connecting measure block to CC %d", cc);
	res = i2c_reg_write_byte_dt(&i2c, REG_SWITCHES0, 0b11 | (cc << 2));
	ERROR_CHECK(res);

	LOG_INF("Flushing TX buffer");
	res = i2c_reg_write_byte_dt(&i2c, REG_CONTROL0, 0x40);
	ERROR_CHECK(res);

	LOG_INF("Flushing RX buffer");
	res = i2c_reg_write_byte_dt(&i2c, REG_CONTROL1, 0x04);
	ERROR_CHECK(res);

	LOG_INF("Resetting PD logic");
	res = i2c_reg_write_byte_dt(&i2c, REG_RESET, 0x02);
	ERROR_CHECK(res);

	// Wait for source caps
	LOG_INF("Waiting for source caps");
	wait_until_message(&i2c);
	struct pd_msg message;
	res = read_message(&i2c, &message);
	if (res <= 0) {
		LOG_ERR("Error during read: %d", res);
		return 1;
	}
	LOG_INF("Received message, length %d", res);

	if (message.type != PD_PACKET_SOP) {
		LOG_ERR("message type != SOP");
		goto fail;
	}

	LOG_INF("Spec revision %d", message.header.specification_revision);
	LOG_INF("%d PDOs received", message.header.number_of_data_objects);


	for (int i = 0; i < message.header.number_of_data_objects; i++) {
		uint32_t pdo = message.data[4 * i] | (message.data[4 * i + 1] << 8) | (message.data[4 * i + 2] << 16) |
					   (message.data[4 * i + 3] << 24);
		handle_pdo(i + 1, pdo);
	}

	// Request cap

	union pd_header header = {0};
	header.message_type = PD_DATA_REQUEST;
	header.port_data_role = TC_ROLE_UFP;
	header.specification_revision = 3;
	header.port_power_role = TC_ROLE_SINK;
	header.message_id = 0;
	header.number_of_data_objects = 1;
	header.extended = 0;

	union pd_rdo rdo = {0};
	rdo.fixed.min_or_max_operating_current = 10;// * 10mA
	rdo.fixed.operating_current = 1;            // * 10mA
	rdo.fixed.unchunked_ext_msg_supported = 0;
	rdo.fixed.no_usb_suspend = 1;
	rdo.fixed.usb_comm_capable = 0;
	rdo.fixed.cap_mismatch = 0;
	rdo.fixed.giveback = 0;
	rdo.fixed.object_pos = 1;

	uint8_t preamble[] = {SOP1, SOP1, SOP1, SOP2, PACKSYM | 6 /* 2 bytes header + 4 bytes RDO */};
	uint8_t header_data[] = {header.raw_value & 0xFF, (header.raw_value >> 8) & 0xFF};
	uint8_t data[] = {rdo.raw_value & 0xFF, (rdo.raw_value >> 8) & 0xFF, (rdo.raw_value >> 16) & 0xFF,
					  (rdo.raw_value >> 24) & 0xFF};
	uint8_t eop_seq[] = {JAM_CRC, EOP, TXOFF, TXON};

	LOG_HEXDUMP_DBG(data, sizeof(data), "RDO data");

	i2c_burst_write_dt(&i2c, REG_FIFO, preamble, sizeof(preamble));
	i2c_burst_write_dt(&i2c, REG_FIFO, header_data, sizeof(header_data));
	i2c_burst_write_dt(&i2c, REG_FIFO, data, sizeof(data));
	i2c_burst_write_dt(&i2c, REG_FIFO, eop_seq, sizeof(eop_seq));

	// (Wait for accept)
	// (Wait for PS RDY)

fail:
	log_panic();
	while (true) { k_msleep(100); }
}
