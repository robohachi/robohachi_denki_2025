#include "bno055.h"

#define BNO055_TIMEOUT_MS  1000

/* Private function prototypes */
static bool BNO055_SetPage(bno055_t *dev, uint8_t page);
static int16_t BNO055_Get16BitData(bno055_t *dev, uint8_t reg_addr);
static bool BNO055_InitHIDI2C(bno055_t *dev);
static bool BNO055_HIDWriteRegister(bno055_t *dev, uint8_t reg, uint8_t value);
static bool BNO055_HIDReadRegister(bno055_t *dev, uint8_t reg, uint8_t *data);

/**
 * @brief Initialize BNO055 sensor (with HID-I2C support)
 * @param dev: BNO055 device structure
 * @param hi2c: I2C handle
 * @param address: I2C address of BNO055
 * @return true if initialization successful, false otherwise
 */
bool BNO055_Init(bno055_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    dev->hi2c = hi2c;
    dev->address = address << 1; // HAL expects 8-bit address
    dev->mode = OPERATION_MODE_CONFIG;

    // Check if device is in HID-I2C mode (address 0x40)
    if (address == BNO055_I2C_ADDR_HID) {
        // Initialize HID-I2C mode
        if (!BNO055_InitHIDI2C(dev)) {
            return false;
        }
    } else {
        // Standard I2C initialization
        // Check if the chip responds
        uint8_t chip_id = BNO055_GetChipID(dev);
        if (chip_id != BNO055_ID) {
            return false;
        }

        // Reset the device
        if (!BNO055_Reset(dev)) {
            return false;
        }

        // Wait for the chip to be ready
        HAL_Delay(1000);
    }

    // Set to normal power mode
    if (!BNO055_SetPowerMode(dev, POWER_MODE_NORMAL)) {
        return false;
    }

    // Set page 0
    if (!BNO055_SetPage(dev, 0)) {
        return false;
    }

    // Set to NDOF mode
    if (!BNO055_SetMode(dev, OPERATION_MODE_NDOF)) {
        return false;
    }

    return true;
}

/**
 * @brief Reset BNO055 sensor
 * @param dev: BNO055 device structure
 * @return true if reset successful, false otherwise
 */
bool BNO055_Reset(bno055_t *dev)
{
    if (!BNO055_WriteData(dev, BNO055_SYS_TRIGGER_ADDR, 0x20)) {
        return false;
    }

    // Wait for reset to complete
    HAL_Delay(1000);

    return true;
}

/**
 * @brief Get chip ID
 * @param dev: BNO055 device structure
 * @return Chip ID value
 */
uint8_t BNO055_GetChipID(bno055_t *dev)
{
    uint8_t chip_id;
    if (BNO055_ReadData(dev, BNO055_CHIP_ID_ADDR, &chip_id)) {
        return chip_id;
    }
    return 0;
}

/**
 * @brief Set operation mode
 * @param dev: BNO055 device structure
 * @param mode: Operation mode to set
 * @return true if successful, false otherwise
 */
bool BNO055_SetMode(bno055_t *dev, bno055_opmode_t mode)
{
    if (!BNO055_WriteData(dev, BNO055_OPR_MODE_ADDR, mode)) {
        return false;
    }

    dev->mode = mode;
    HAL_Delay(30); // Wait for mode change

    return true;
}

/**
 * @brief Get current operation mode
 * @param dev: BNO055 device structure
 * @return Current operation mode
 */
bno055_opmode_t BNO055_GetMode(bno055_t *dev)
{
    uint8_t mode;
    if (BNO055_ReadData(dev, BNO055_OPR_MODE_ADDR, &mode)) {
        return (bno055_opmode_t)mode;
    }
    return OPERATION_MODE_CONFIG;
}

/**
 * @brief Set power mode
 * @param dev: BNO055 device structure
 * @param mode: Power mode to set
 * @return true if successful, false otherwise
 */
bool BNO055_SetPowerMode(bno055_t *dev, bno055_powermode_t mode)
{
    return BNO055_WriteData(dev, BNO055_PWR_MODE_ADDR, mode);
}

/**
 * @brief Get accelerometer data
 * @param dev: BNO055 device structure
 * @return Accelerometer vector (m/s²)
 */
bno055_vector_t BNO055_GetVectorAccelerometer(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_ACCEL_DATA_X_LSB_ADDR) / 100.0f;
    vector.y = BNO055_Get16BitData(dev, BNO055_ACCEL_DATA_Y_LSB_ADDR) / 100.0f;
    vector.z = BNO055_Get16BitData(dev, BNO055_ACCEL_DATA_Z_LSB_ADDR) / 100.0f;

    return vector;
}

/**
 * @brief Get magnetometer data
 * @param dev: BNO055 device structure
 * @return Magnetometer vector (µT)
 */
bno055_vector_t BNO055_GetVectorMagnetometer(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_MAG_DATA_X_LSB_ADDR) / 16.0f;
    vector.y = BNO055_Get16BitData(dev, BNO055_MAG_DATA_Y_LSB_ADDR) / 16.0f;
    vector.z = BNO055_Get16BitData(dev, BNO055_MAG_DATA_Z_LSB_ADDR) / 16.0f;

    return vector;
}

/**
 * @brief Get gyroscope data
 * @param dev: BNO055 device structure
 * @return Gyroscope vector (rad/s)
 */
bno055_vector_t BNO055_GetVectorGyroscope(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_GYRO_DATA_X_LSB_ADDR) / 16.0f;
    vector.y = BNO055_Get16BitData(dev, BNO055_GYRO_DATA_Y_LSB_ADDR) / 16.0f;
    vector.z = BNO055_Get16BitData(dev, BNO055_GYRO_DATA_Z_LSB_ADDR) / 16.0f;

    return vector;
}

/**
 * @brief Get Euler angles
 * @param dev: BNO055 device structure
 * @return Euler angles vector (degrees)
 */
bno055_vector_t BNO055_GetVectorEuler(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_EULER_H_LSB_ADDR) / 16.0f; // Heading
    vector.y = BNO055_Get16BitData(dev, BNO055_EULER_R_LSB_ADDR) / 16.0f; // Roll
    vector.z = BNO055_Get16BitData(dev, BNO055_EULER_P_LSB_ADDR) / 16.0f; // Pitch

    return vector;
}

/**
 * @brief Get linear acceleration data
 * @param dev: BNO055 device structure
 * @return Linear acceleration vector (m/s²)
 */
bno055_vector_t BNO055_GetVectorLinearAccel(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR) / 100.0f;
    vector.y = BNO055_Get16BitData(dev, BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR) / 100.0f;
    vector.z = BNO055_Get16BitData(dev, BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR) / 100.0f;

    return vector;
}

/**
 * @brief Get gravity vector
 * @param dev: BNO055 device structure
 * @return Gravity vector (m/s²)
 */
bno055_vector_t BNO055_GetVectorGravity(bno055_t *dev)
{
    bno055_vector_t vector = {0};

    vector.x = BNO055_Get16BitData(dev, BNO055_GRAVITY_DATA_X_LSB_ADDR) / 100.0f;
    vector.y = BNO055_Get16BitData(dev, BNO055_GRAVITY_DATA_Y_LSB_ADDR) / 100.0f;
    vector.z = BNO055_Get16BitData(dev, BNO055_GRAVITY_DATA_Z_LSB_ADDR) / 100.0f;

    return vector;
}

/**
 * @brief Get quaternion data
 * @param dev: BNO055 device structure
 * @return Quaternion data
 */
bno055_quaternion_t BNO055_GetQuaternion(bno055_t *dev)
{
    bno055_quaternion_t quat = {0};

    quat.w = BNO055_Get16BitData(dev, BNO055_QUATERNION_DATA_W_LSB_ADDR) / 16384.0f;
    quat.x = BNO055_Get16BitData(dev, BNO055_QUATERNION_DATA_X_LSB_ADDR) / 16384.0f;
    quat.y = BNO055_Get16BitData(dev, BNO055_QUATERNION_DATA_Y_LSB_ADDR) / 16384.0f;
    quat.z = BNO055_Get16BitData(dev, BNO055_QUATERNION_DATA_Z_LSB_ADDR) / 16384.0f;

    return quat;
}

/**
 * @brief Get calibration status
 * @param dev: BNO055 device structure
 * @return Calibration status structure
 */
bno055_calibration_t BNO055_GetCalibration(bno055_t *dev)
{
    bno055_calibration_t cal = {0};
    uint8_t calData;

    if (BNO055_ReadData(dev, BNO055_CALIB_STAT_ADDR, &calData)) {
        cal.sys = (calData >> 6) & 0x03;
        cal.gyro = (calData >> 4) & 0x03;
        cal.accel = (calData >> 2) & 0x03;
        cal.mag = calData & 0x03;
    }

    return cal;
}

/**
 * @brief Check if sensor is fully calibrated
 * @param dev: BNO055 device structure
 * @return true if fully calibrated, false otherwise
 */
bool BNO055_IsFullyCalibrated(bno055_t *dev)
{
    bno055_calibration_t cal = BNO055_GetCalibration(dev);
    return (cal.sys == 3 && cal.gyro == 3 && cal.accel == 3 && cal.mag == 3);
}

/**
 * @brief Get temperature
 * @param dev: BNO055 device structure
 * @return Temperature in Celsius
 */
int8_t BNO055_GetTemp(bno055_t *dev)
{
    uint8_t temp;
    if (BNO055_ReadData(dev, BNO055_TEMP_ADDR, &temp)) {
        return (int8_t)temp;
    }
    return 0;
}

/**
 * @brief Get system status
 * @param dev: BNO055 device structure
 * @return System status value
 */
uint8_t BNO055_GetSystemStatus(bno055_t *dev)
{
    uint8_t sys_stat;
    if (BNO055_ReadData(dev, BNO055_SYS_STAT_ADDR, &sys_stat)) {
        return sys_stat;
    }
    return 0;
}

/**
 * @brief Get system error
 * @param dev: BNO055 device structure
 * @return System error value
 */
uint8_t BNO055_GetSystemError(bno055_t *dev)
{
    uint8_t sys_err;
    if (BNO055_ReadData(dev, BNO055_SYS_ERR_ADDR, &sys_err)) {
        return sys_err;
    }
    return 0;
}

/**
 * @brief Set axis remap configuration
 * @param dev: BNO055 device structure
 * @param remap_config: Axis remap configuration
 * @param remap_sign: Axis remap sign
 * @return true if successful, false otherwise
 */
bool BNO055_SetAxisRemap(bno055_t *dev, bno055_axis_remap_config_t remap_config, bno055_axis_remap_sign_t remap_sign)
{
    bno055_opmode_t mode = BNO055_GetMode(dev);

    if (!BNO055_SetMode(dev, OPERATION_MODE_CONFIG)) {
        return false;
    }

    if (!BNO055_WriteData(dev, BNO055_AXIS_MAP_CONFIG_ADDR, remap_config)) {
        return false;
    }

    if (!BNO055_WriteData(dev, BNO055_AXIS_MAP_SIGN_ADDR, remap_sign)) {
        return false;
    }

    return BNO055_SetMode(dev, mode);
}

/**
 * @brief Initialize HID-I2C communication with BNO055
 * @param dev: BNO055 device structure
 * @return true if successful, false otherwise
 */
static bool BNO055_InitHIDI2C(bno055_t *dev)
{
    // HID-I2C Reset command
    uint8_t reset_cmd[] = {0x01, 0x00};  // Reset command
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, reset_cmd, 2, 1000);
    if (status != HAL_OK) {
        return false;
    }

    HAL_Delay(1000); // Wait for reset

    // Try to read chip ID through HID-I2C protocol
    uint8_t chip_id;
    if (!BNO055_HIDReadRegister(dev, BNO055_CHIP_ID_ADDR, &chip_id)) {
        return false;
    }

    if (chip_id != BNO055_ID) {
        return false;
    }

    return true;
}

/**
 * @brief Write to BNO055 register using HID-I2C protocol
 * @param dev: BNO055 device structure
 * @param reg: Register address
 * @param value: Value to write
 * @return true if successful, false otherwise
 */
static bool BNO055_HIDWriteRegister(bno055_t *dev, uint8_t reg, uint8_t value)
{
    // HID-I2C write packet format
    uint8_t write_cmd[] = {
        0x03,           // Report ID for feature report
        0x04,           // Length of data
        reg,            // Register address
        value,          // Data to write
        0x00            // Padding
    };

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, write_cmd, sizeof(write_cmd), 1000);
    return (status == HAL_OK);
}

/**
 * @brief Read from BNO055 register using HID-I2C protocol
 * @param dev: BNO055 device structure
 * @param reg: Register address
 * @param data: Pointer to store read data
 * @return true if successful, false otherwise
 */
static bool BNO055_HIDReadRegister(bno055_t *dev, uint8_t reg, uint8_t *data)
{
    // HID-I2C read request packet
    uint8_t read_cmd[] = {
        0x03,           // Report ID for feature report
        0x03,           // Length of data
        reg,            // Register address
        0x01            // Number of bytes to read
    };

    // Send read request
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, read_cmd, sizeof(read_cmd), 1000);
    if (status != HAL_OK) {
        return false;
    }

    HAL_Delay(10); // Small delay for processing

    // Read response
    uint8_t response[8];
    status = HAL_I2C_Master_Receive(dev->hi2c, dev->address, response, sizeof(response), 1000);
    if (status != HAL_OK) {
        return false;
    }

    // Extract data from HID response
    if (response[0] == 0x01) {  // Input report ID
        *data = response[3];    // Data is typically at offset 3
        return true;
    }

    return false;
}

/**
 * @brief Write single byte to register (with HID-I2C support)
 * @param dev: BNO055 device structure
 * @param reg: Register address
 * @param value: Value to write
 * @return true if successful, false otherwise
 */
bool BNO055_WriteData(bno055_t *dev, uint8_t reg, uint8_t value)
{
    if ((dev->address >> 1) == BNO055_I2C_ADDR_HID) {
        return BNO055_HIDWriteRegister(dev, reg, value);
    } else {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, BNO055_TIMEOUT_MS);
        return (status == HAL_OK);
    }
}

/**
 * @brief Read single byte from register (with HID-I2C support)
 * @param dev: BNO055 device structure
 * @param reg: Register address
 * @param data: Pointer to store read data
 * @return true if successful, false otherwise
 */
bool BNO055_ReadData(bno055_t *dev, uint8_t reg, uint8_t *data)
{
    if ((dev->address >> 1) == BNO055_I2C_ADDR_HID) {
        return BNO055_HIDReadRegister(dev, reg, data);
    } else {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, 1, BNO055_TIMEOUT_MS);
        return (status == HAL_OK);
    }
}

/**
 * @brief Read multiple bytes from registers (with HID-I2C support)
 * @param dev: BNO055 device structure
 * @param reg: Starting register address
 * @param data: Data buffer to store read data
 * @param length: Number of bytes to read
 * @return true if successful, false otherwise
 */
bool BNO055_ReadMultiple(bno055_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if ((dev->address >> 1) == BNO055_I2C_ADDR_HID) {
        // For HID-I2C, read bytes one by one (HID protocol limitation)
        for (uint8_t i = 0; i < length; i++) {
            if (!BNO055_HIDReadRegister(dev, reg + i, &data[i])) {
                return false;
            }
            HAL_Delay(1); // Small delay between reads
        }
        return true;
    } else {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, length, BNO055_TIMEOUT_MS);
        return (status == HAL_OK);
    }
}

/**
 * @brief Write multiple bytes to registers (with HID-I2C support)
 * @param dev: BNO055 device structure
 * @param reg: Starting register address
 * @param data: Data buffer to write
 * @param length: Number of bytes to write
 * @return true if successful, false otherwise
 */
bool BNO055_WriteMultiple(bno055_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if ((dev->address >> 1) == BNO055_I2C_ADDR_HID) {
        // For HID-I2C, write bytes one by one
        for (uint8_t i = 0; i < length; i++) {
            if (!BNO055_HIDWriteRegister(dev, reg + i, data[i])) {
                return false;
            }
            HAL_Delay(1); // Small delay between writes
        }
        return true;
    } else {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, length, BNO055_TIMEOUT_MS);
        return (status == HAL_OK);
    }
}

/* Private Functions */

/**
 * @brief Set register page
 * @param dev: BNO055 device structure
 * @param page: Page number to set
 * @return true if successful, false otherwise
 */
static bool BNO055_SetPage(bno055_t *dev, uint8_t page)
{
    return BNO055_WriteData(dev, BNO055_PAGE_ID_ADDR, page);
}

/**
 * @brief Read 16-bit data from registers
 * @param dev: BNO055 device structure
 * @param reg_addr: LSB register address
 * @return 16-bit signed data
 */
static int16_t BNO055_Get16BitData(bno055_t *dev, uint8_t reg_addr)
{
    uint8_t buffer[2];

    if (BNO055_ReadMultiple(dev, reg_addr, buffer, 2)) {
        return (int16_t)((buffer[1] << 8) | buffer[0]);
    }

    return 0;
}
