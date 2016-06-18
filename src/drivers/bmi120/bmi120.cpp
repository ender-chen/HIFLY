#include <sys/types.h>
#include <stdlib.h>

#include <systemlib/perf_counter.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
//#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#define DIR_READ                        0x80
#define DIR_WRITE						0x00

#define BMI_DEVICE_PATH_ACCEL		"/dev/bmi120_accel"
#define BMI_DEVICE_PATH_GYRO		"/dev/bmi120_gyro"

// BMI120 registers
#define BMIREG_WHOAMI					0x00
#define BMIREG_ERR						0x02
#define BMIREG_PMU_STATUS				0x03
#define BMIREG_GYRO_XOUT_L				0x0C
#define BMIREG_GYRO_XOUT_H				0x0D
#define BMIREG_GYRO_YOUT_L				0x0E
#define BMIREG_GYRO_YOUT_H				0x0F
#define BMIREG_GYRO_ZOUT_L				0x10
#define BMIREG_GYRO_ZOUT_H				0x11
#define BMIREG_ACCEL_XOUT_L				0x12
#define BMIREG_ACCEL_XOUT_H				0x13
#define BMIREG_ACCEL_YOUT_L				0x14
#define BMIREG_ACCEL_YOUT_H				0x15
#define BMIREG_ACCEL_ZOUT_L				0x16
#define BMIREG_ACCEL_ZOUT_H				0x17
#define BMIREG_STATUR					0x1B
#define BMIREG_TEMP_L					0x20
#define BMIREG_TEMP_H					0x21
#define BMIREG_ACC_CONF					0x40
#define BMIREG_ACC_RANGE				0x41
#define BMIREG_GYRO_CONF				0x42
#define BMIREG_GYRO_RANGE				0x43
#define BMIREG_INT_EN_0					0x50
#define BMIREG_INT_EN_1					0x51
#define BMIREG_INT_EN_2					0x52
#define BMIREG_INT_OUT_CTRL				0x53
#define BMIREG_IF_CONF					0x6B
#define BMIREG_PMU_TRIGGER				0x6C
#define BMIREG_SELF_TEST				0x6D
#define BMIREG_USER_NV_CONFIG_ADDR      0x70
#define BMIREG_OFFSET_0					0x71
#define BMIREG_OFFSET_1					0x72
#define BMIREG_OFFSET_2					0x73
#define BMIREG_OFFSET_3					0x74
#define BMIREG_OFFSET_4					0x75
#define BMIREG_OFFSET_5					0x76
#define BMIREG_OFFSET_6					0x77
#define BMIREG_STEP_CONF_0				0x7A
#define BMIREG_STEP_CONF_1				0x7B
#define BMIREG_CMD						0x7E

// Configuration bits BMI 120
#define BITS_SPI_EN						0x01
#define BITS_WDT_SEL					0x02
#define BITS_I2C_WDT_EN					0x04

#define BITS_GYRO_ODR_RESERVED			0x00
#define BITS_GYRO_ODR_25HZ				0x06
#define BITS_GYRO_ODR_50HZ				0x07
#define	BITS_GYRO_ODR_100HZ				0x08
#define BITS_GYRO_ODR_200HZ				0x09
#define BITS_GYRO_ODR_400HZ				0x0A
#define BITS_GYRO_ODR_800HZ				0x0B
#define BITS_GYRO_ODR_1600HZ			0x0C
#define BITS_GYRO_ODR_3200HZ			0x0D

#define BITS_GYRO_RANGE_2000			0
#define BITS_GYRO_RANGE_1000			1
#define BITS_GYRO_RANGE_500				2
#define BITS_GYRO_RANGE_250				3
#define BITS_GYRO_RANGE_125				4

#define BITS_ACCEL_ODR_RESERVED			0x00
#define BITS_ACCEL_ODR_0_78HZ			0x01
#define BITS_ACCEL_ODR_1_56HZ			0x02
#define BITS_ACCEL_ODR_3_12HZ			0x03
#define BITS_ACCEL_ODR_6_26HZ			0x04
#define BITS_ACCEL_ODR_12_5HZ			0x05
#define BITS_ACCEL_ODR_25HZ				0x06
#define BITS_ACCEL_ODR_50HZ				0x07
#define BITS_ACCEL_ODR_100HZ			0x08
#define BITS_ACCEL_ODR_200HZ			0x09
#define BITS_ACCEL_ODR_400HZ			0x0A
#define BITS_ACCEL_ODR_800HZ			0x0B
#define BITS_ACCEL_ODR_1600HZ			0x0C
#define BITS_ACCEL_ODR_RESERVED0		0x0D
#define BITS_ACCEL_ODR_RESERVED1		0x0E
#define BITS_ACCEL_ODR_RESERVED2		0x0F

#define BITS_ACCEL_RANGE_2G				0x03
#define BITS_ACCEL_RANGE_4G				0x05
#define BITS_ACCEL_RANGE_8G				0x08
#define BITS_ACCEL_RANGE_16G			0x0C

#define CMD_PMU_GYRO_SUSPEND			0x14 //Sets the PMU mode for the Gyroscope to suspend
#define CMD_PMU_GYRO_NORMAL				0x15 //Sets the PMU mode for the Gyroscope to normal
#define CMD_PMU_GYRO_FASTSTART			0x17 //Sets the PMU mode for the Gyroscope to fast start-up

#define CMD_PMU_ACC_SUSPEND				0x10 //Sets the PMU mode for the Accelerometer to suspend
#define CMD_PMU_ACC_NORMAL				0x11 //Sets the PMU mode for the Accelerometer to normal
#define CMD_PMU_ACC_LOWPOWER			0x12 //Sets the PMU mode for the Accelerometer Lowpower

#define CMD_PMU_RESET					0xB6 //Triggers a reset

#define BMI120_WHOAMI_ID				0xD3 //the chip identification code

#define BMI120_ACCEL_DEFAULT_RATE      800
#define BMI120_ACCEL_MAX_OUTPUT_RATE                   280
#define BMI120_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define BMI120_GYRO_DEFAULT_RATE       800
#define BMI120_GYRO_MAX_OUTPUT_RATE                    BMI120_ACCEL_MAX_OUTPUT_RATE
#define BMI120_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define BMI120_ONE_G                   9.80665f

#define BMI120_LOW_BUS_SPEED                           1000*1000
#define BMI120_HIGH_BUS_SPEED                          11*1000*1000

#define BMI120_TIMER_REDUCTION                         200

class BMI120_gyro;

class BMI120 : public device::SPI
{
public:
	BMI120(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~BMI120();

	virtual int		init();
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void                    print_info();

	void                    print_registers();
protected:
	virtual int		probe();

	friend class BMI120_gyro;

	virtual ssize_t         gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int             gyro_ioctl(struct file *filp, int cmd, unsigned long arg);
private:
	BMI120_gyro            *_gyro;
	uint8_t 		_whoami;

	struct hrt_call         _call;
	unsigned                _call_interval;

	ringbuffer::RingBuffer  *_accel_reports;

	struct accel_scale  _accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t        _accel_topic;
	int         _accel_orb_class_instance;
	int         _accel_class_instance;

	ringbuffer::RingBuffer  *_gyro_reports;
	struct gyro_scale   _gyro_scale;
	float           _gyro_range_scale;
	float           _gyro_range_rad_s;


	perf_counter_t      _accel_reads;
	perf_counter_t      _gyro_reads;
	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _reset_retries;
	perf_counter_t      _duplicates;
	perf_counter_t      _controller_latency_perf;

	uint8_t         _register_wait;
	uint64_t        _reset_wait;

	math::LowPassFilter2p   _accel_filter_x;
	math::LowPassFilter2p   _accel_filter_y;
	math::LowPassFilter2p   _accel_filter_z;
	math::LowPassFilter2p   _gyro_filter_x;
	math::LowPassFilter2p   _gyro_filter_y;
	math::LowPassFilter2p   _gyro_filter_z;

	Integrator              _accel_int;
	Integrator              _gyro_int;

	enum Rotation       _rotation;
#define BMI120_NUM_CHECKED_REGISTERS 5
	static const uint8_t    _checked_registers[BMI120_NUM_CHECKED_REGISTERS];
	uint8_t                 _checked_values[BMI120_NUM_CHECKED_REGISTERS];
	uint8_t                 _checked_bad[BMI120_NUM_CHECKED_REGISTERS];
	uint8_t                 _checked_next;

	// last temperature reading for print_info()
	float           _last_temperature;

	// keep last accel reading for duplicate detection
	uint16_t        _last_accel[3];
	bool            _got_duplicate;

	/**
	* Start automatic measurement.
	*/
	void                    start();
	/**
	* Stop automatic measurement.
	*/
	void                    stop();
	/**
	* Reset chip.
	×
	* Resets the chip and measurements ranges, but not scale and offset.
	*/
	int			reset();
	/**
	* Static trampoline from the hrt_call context; because we don't have a
	* generic hrt wrapper yet.
	*
	* Called by the HRT in interrupt context at the specified rate if
	* automatic polling is enabled.
	*
	* @param arg           Instance pointer for the driver that is polling.
	*/
	static void             measure_trampoline(void *arg);
	/**
	* Fetch measurements from the sensor and update the report ring.
	*/
	void		measure();
	/**
	* Read a register from the BMI120
	*
	* @param               The register to read.
	* @return              The value that was read.
	*/
	uint8_t                 read_reg(unsigned reg, uint32_t speed = BMI120_LOW_BUS_SPEED);
	uint16_t                read_reg16(unsigned reg);
	/**
	* Write a register in the BMI120
	*
	* @param reg           The register to write.
	* @param value         The new value to write.
	*/
	void                    write_reg(unsigned reg, uint8_t value);
	/**
	 * Modify a register in the BMI120
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg           The register to modify.
	 * @param clearbits     Bits in the register to clear.
	 * @param setbits       Bits in the register to set.
	 */
	void                    modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);
	/**
	* Write a register in the BMI120, updating _checked_values
	*
	* @param reg           The register to write.
	* @param value         The new value to write.
	*/
	void                    write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the BMI120 measurement range.
	 *
	 * @param max_g         The maximum G value the range must support.
	 * @return              OK if the value can be supported, -ERANGE otherwise.
	 */
	int                     set_accel_range(unsigned max_g);
	/**
	 * Swap a 16-bit value read from the BMI120 to native byte order.
	 */
	uint16_t                swap16(uint16_t val) { return (val >> 8) | (val << 8);  }
	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int                     self_test();
	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int                     accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int                     gyro_self_test();
	/*
	 check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI120(const BMI120 &);
	BMI120 operator=(const BMI120 &);

#pragma pack(push, 1)
	/**
	 * Report conversation within the BMI120, including command byte and
	 * interrupt status.
	 */
	struct BMIReport {
		uint8_t cmd;
		uint8_t gyro_x[2];
		uint8_t gyro_y[2];
		uint8_t gyro_z[2];
		uint8_t accel_x[2];
		uint8_t accel_y[2];
		uint8_t accel_z[2];
		uint8_t sensor_time[3];
		uint8_t status;
		uint8_t int_status[4];
		uint8_t temp[2];
	};
#pragma pack(pop)
};
/*
* list of registers that will be checked in check_registers(). Note
* that MPUREG_PRODUCT_ID must be first in the list.
*/
const uint8_t BMI120::_checked_registers[BMI120_NUM_CHECKED_REGISTERS] = {
									  BMIREG_WHOAMI,
									  BMIREG_ACC_CONF,
									  BMIREG_GYRO_CONF,
									  BMIREG_GYRO_RANGE,
									  BMIREG_ACC_RANGE
									 };

/**
 * Helper class implementing the gyro driver node.
 */
class BMI120_gyro : public device::CDev
{
public:
	BMI120_gyro(BMI120 *parent, const char *path);
	~BMI120_gyro();

	virtual ssize_t         read(struct file *filp, char *buffer, size_t buflen);
	virtual int             ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int             init();
protected:
	friend class BMI120;

	void                    parent_poll_notify();
private:
	BMI120                 *_parent;
	orb_advert_t            _gyro_topic;
	int                     _gyro_orb_class_instance;
	int                     _gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	BMI120_gyro(const BMI120_gyro &);
	BMI120_gyro operator=(const BMI120_gyro &);
};

/** driver 'main' command */
extern "C" {__EXPORT int bmi120_main(int argc, char *argv[]); }

BMI120::BMI120(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
	SPI("BMI120", path_accel, bus, device, SPIDEV_MODE3, BMI120_LOW_BUS_SPEED),
	_gyro(new BMI120_gyro(this, path_gyro)),
	_whoami(0),
	_call{},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_accel_reads(perf_alloc(PC_COUNT, "bmi120_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "bmi120_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi120_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi120_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi120_bad_registers")),
	_good_transfers(perf_alloc(PC_COUNT, "bmi120_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "bmi120_reset_retries")),
	_duplicates(perf_alloc(PC_COUNT, "bmi120_duplicates")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(BMI120_ACCEL_DEFAULT_RATE, BMI120_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(BMI120_ACCEL_DEFAULT_RATE, BMI120_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(BMI120_ACCEL_DEFAULT_RATE, BMI120_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(BMI120_GYRO_DEFAULT_RATE, BMI120_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(BMI120_GYRO_DEFAULT_RATE, BMI120_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(BMI120_GYRO_DEFAULT_RATE, BMI120_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / BMI120_ACCEL_MAX_OUTPUT_RATE),
	_gyro_int(1000000 / BMI120_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_checked_next(0),
	_last_temperature(0),
	_last_accel{},
	_got_duplicate(false)
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_BMI120;

	/* Prime _gyro with parents devid. */
	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_BMI120;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

BMI120::~BMI120()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
BMI120::init()
{
	int ret = OK;

	/*do SPI init (and probe) first*/
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;


	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_HIGH - 1);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance,  ORB_PRIO_HIGH - 1);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

void
BMI120::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	set_frequency(BMI120_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));
}

uint8_t
BMI120::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
BMI120::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	// general register transfer at low clock speed
	set_frequency(BMI120_LOW_BUS_SPEED);

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
BMI120::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI120_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI120::reset()
{
	write_reg(BMIREG_CMD, CMD_PMU_RESET);
	up_udelay(10000);

	write_reg(BMIREG_USER_NV_CONFIG_ADDR, BITS_SPI_EN);
	up_udelay(1000);

	write_reg(BMIREG_CMD, CMD_PMU_ACC_NORMAL);
	up_udelay(1000);

	write_reg(BMIREG_CMD, CMD_PMU_GYRO_NORMAL);
	up_udelay(1000);

	write_checked_reg(BMIREG_ACC_CONF, BITS_ACCEL_ODR_800HZ);
	usleep(1000);

	write_checked_reg(BMIREG_GYRO_CONF, BITS_GYRO_ODR_800HZ);
	usleep(1000);

	// Gyro scale 2000 deg/s ()
	write_checked_reg(BMIREG_GYRO_RANGE, BITS_GYRO_RANGE_2000);
	usleep(1000);

	_gyro_range_scale = (0.0174532 / 16.4);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(8);
	usleep(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI120_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
					write_reg(_checked_registers[i], _checked_values[i]);
					all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
BMI120::set_accel_range(unsigned max_g_in)
{
	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	 if (max_g_in > 8) { // 16g
		afs_sel = BITS_ACCEL_RANGE_16G;
		lsb_per_g = 2048;
		max_accel_g = 16;
	 } else if (max_g_in > 4) { //  8g
		afs_sel = BITS_ACCEL_RANGE_8G;
		lsb_per_g = 4096;
		max_accel_g = 8;
	 } else if (max_g_in > 2) { //  4g
		afs_sel = BITS_ACCEL_RANGE_4G;
		lsb_per_g = 8192;
		max_accel_g = 4;
	 } else {                //  2g
		afs_sel = BITS_ACCEL_RANGE_2G;
		lsb_per_g = 16384;
		max_accel_g = 2;
	 }

	 write_checked_reg(BMIREG_ACC_RANGE, afs_sel);
	 _accel_range_scale = (BMI120_ONE_G / lsb_per_g);
	 _accel_range_m_s2 = max_accel_g * BMI120_ONE_G;

	 return OK;
}

int
BMI120::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMIREG_WHOAMI);

	// verify product revision
	switch (_whoami) {
		case BMI120_WHOAMI_ID:
			memset(_checked_values, 0, sizeof(_checked_values));
			memset(_checked_bad, 0, sizeof(_checked_bad));
			_checked_values[0] = _whoami;
			_checked_bad[0] = _whoami;
			return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

ssize_t
BMI120::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
BMI120::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
BMI120::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f) {
		return 1;
	}

	if (fabsf(_accel_scale.y_offset) < 0.000001f) {
		return 1;
	}

	if (fabsf(_accel_scale.z_offset) < 0.000001f) {
		return 1;
	}

	return 0;
}

int
BMI120::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	/*
	 * Maximum deviation of 20 degrees, according to
	 * http://www.invensense.com/mems/gyro/documents/PS-MPU-9250A-00v3.4.pdf
	 * Section 6.1, initial ZRO tolerance
	 */
	const float max_offset = 0.34f;
	/* 30% scale error is chosen to catch completely faulty units but
	 * to let some slight scale error pass. Requires a rate table or correlation
	 * with mag rotations + data fit to
	 * calibrate properly and is not done by default.
	 */
	const float max_scale = 0.3f;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 20 dps. */
	if (fabsf(_gyro_scale.x_offset) > max_offset) {
		return 1;
	}

	/* evaluate gyro scale, complain if off by more than 30% */
	if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
		return 1;
	}

	/* check if all scales are zero */
	if ((fabsf(_gyro_scale.x_offset) < 0.000001f) &&
	    (fabsf(_gyro_scale.y_offset) < 0.000001f) &&
	    (fabsf(_gyro_scale.z_offset) < 0.000001f)) {
		/* if all are zero, this device is not calibrated */
		return 1;
	}

	return 0;
}


ssize_t
BMI120::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}


int
BMI120::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, BMI120_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

					/*
					  set call interval faster then the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the bmi120 clock
					 */
					_call.period = _call_interval - BMI120_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_accel_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case ACCELIOCGLOWPASS:
		return _accel_filter_x.get_cutoff_freq();

	case ACCELIOCSLOWPASS:
		// set software filtering
		_accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_scale *s = (struct accel_scale *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return set_accel_range(arg);

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / BMI120_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

#ifdef ACCELIOCSHWLOWPASS

	case ACCELIOCSHWLOWPASS:
		_set_dlpf_filter(arg);
		return OK;
#endif

#ifdef ACCELIOCGHWLOWPASS

	case ACCELIOCGHWLOWPASS:
		return _dlpf_freq;
#endif


	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
BMI120::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_gyro_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _gyro_reports->size();

	case GYROIOCGLOWPASS:
		return _gyro_filter_x.get_cutoff_freq();

	case GYROIOCSLOWPASS:
		// set software filtering
		_gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _gyro_range_scale = xx
		// _gyro_range_rad_s = xx
		return -EINVAL;

	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

#ifdef GYROIOCSHWLOWPASS

	case GYROIOCSHWLOWPASS:
		_set_dlpf_filter(arg);
		return OK;
#endif

#ifdef GYROIOCGHWLOWPASS

	case GYROIOCGHWLOWPASS:
		return _dlpf_freq;
#endif

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

void
BMI120::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
BMI120::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       1000,
		       _call_interval - BMI120_TIMER_REDUCTION,
		       (hrt_callout)&BMI120::measure_trampoline, this);
}

void
BMI120::stop()
{
	hrt_cancel(&_call);
}


void
BMI120::measure_trampoline(void *arg)
{
	BMI120 *dev = reinterpret_cast<BMI120 *>(arg);

	/* make another measurement */
	dev->measure();
}

void
BMI120::check_registers(void)
{
	/*
	  we read the register at full speed, even though it isn't
	  listed as a high speed register. The low speed requirement
	  for some registers seems to be a propgation delay
	  requirement for changing sensor configuration, which should
	  not apply to reading a single register. It is also a better
	  test of SPI bus health to read at the same speed as we read
	  the data registers.
	*/
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next], BMI120_HIGH_BUS_SPEED)) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(BMIREG_CMD, CMD_PMU_RESET);
			up_udelay(10000);

			write_reg(BMIREG_USER_NV_CONFIG_ADDR, BITS_SPI_EN);
			up_udelay(1000);

			write_reg(BMIREG_CMD, CMD_PMU_ACC_NORMAL);
			up_udelay(1000);

			write_reg(BMIREG_CMD, CMD_PMU_GYRO_NORMAL);

			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the mpu9250 in a
			// bizarre state where it has all correct
			// register values but large offsets on the
			// accel axes
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI120_NUM_CHECKED_REGISTERS;
}

void
BMI120::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct BMIReport bmi_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI120 in one pass.
	 */
	bmi_report.cmd = DIR_READ | BMIREG_GYRO_XOUT_L;

	// sensor transfer at high clock speed
	set_frequency(BMI120_HIGH_BUS_SPEED);

	if (OK != transfer((uint8_t *)&bmi_report, ((uint8_t *)&bmi_report), sizeof(bmi_report))) {
		return;
	}

	check_registers();

	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (memcmp(&bmi_report.accel_x[0], &_last_accel[0], 6) == 0) {
		// it isn't new data - wait for next timer
		_got_duplicate = true;
	} else {
		_got_duplicate = false;
	}

	memcpy(&_last_accel[0], &bmi_report.accel_x[0], 6);

	if(_got_duplicate) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

	/*
	 * Convert from big to little endian
	 */

	report.accel_x = swap16(int16_t_from_bytes(bmi_report.accel_x));
	report.accel_y = swap16(int16_t_from_bytes(bmi_report.accel_y));
	report.accel_z = swap16(int16_t_from_bytes(bmi_report.accel_z));

	report.temp = swap16(int16_t_from_bytes(bmi_report.temp));

	report.gyro_x = swap16(int16_t_from_bytes(bmi_report.gyro_x));
	report.gyro_y = swap16(int16_t_from_bytes(bmi_report.gyro_y));
	report.gyro_z = swap16(int16_t_from_bytes(bmi_report.gyro_z));

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the mpu6k does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	perf_count(_good_transfers);

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again. We still increment
		// _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}


	/*
	 * Swap axes and negate y
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	/*
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	if (report.temp & 0x8000)
	{
		_last_temperature = (report.temp & 0x7FFF) / 512.0f - 41.0f;
	} else {
		_last_temperature = (report.temp) / 512.0f + 23.0f;
	}

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	math::Vector<3> gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI120::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI120_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i], BMI120_HIGH_BUS_SPEED);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
}

void
BMI120::print_registers()
{
	printf("BMI120 registers\n");

	for (uint8_t reg = 0; reg <= 126; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg % 13 == 0) {
			printf("\n");
		}
	}
}

BMI120_gyro::BMI120_gyro(BMI120 *parent, const char *path) :
	CDev("BMI120_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
}

BMI120_gyro::~BMI120_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
BMI120_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	return ret;
}

void
BMI120_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
BMI120_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
BMI120_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}

namespace bmi120
{
BMI120 *g_dev_int; // on internal bus

void    start(enum Rotation);
void	test();
void	reset();
void	info();
void    stop();
void	usage();

void
start(enum Rotation rotation)
{
	int fd;
	BMI120 **g_dev_ptr = &g_dev_int;
	const char *path_accel = BMI_DEVICE_PATH_ACCEL;
	const char *path_gyro  = BMI_DEVICE_PATH_GYRO;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	*g_dev_ptr = new BMI120(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_MPU, rotation);

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete(*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	const char *path_accel = BMI_DEVICE_PATH_ACCEL;
	const char *path_gyro  = BMI_DEVICE_PATH_GYRO;
	accel_report a_report;
	gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'mpu9250 start')",
		    path_accel);

	/* get the driver */
	int fd_gyro = open(path_gyro, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", path_gyro);
	}

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	warnx("single read");
	warnx("time:     %lld", a_report.timestamp);
	warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
	warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
	      (double)(a_report.range_m_s2 / BMI120_ONE_G));

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
	warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);

	/* reset to default polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd);
	close(fd_gyro);

	/* XXX add poll-rate tests here too */

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	const char *path_accel = BMI_DEVICE_PATH_ACCEL;
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	BMI120 **g_dev_ptr = &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

void
stop()
{
	BMI120 **g_dev_ptr = &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset'");
	warnx("options:");
	warnx("    -R rotation");
}
}//namespace

int
bmi120_main(int argc, char *argv[]) {
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
			switch (ch) {
				case 'R':
					rotation = (enum Rotation)atoi(optarg);
					break;
				default:
					exit(0);
			}
	}
	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		bmi120::start(rotation);
	}

	if (!strcmp(verb, "stop")) {
		bmi120::stop();
	}

	if (!strcmp(verb, "test")) {
		bmi120::test();
	}

	if (!strcmp(verb, "reset")) {
		bmi120::reset();
	}

	if (!strcmp(verb, "info")) {
		bmi120::info();
	}

	bmi120::usage();
	exit(1);
}

