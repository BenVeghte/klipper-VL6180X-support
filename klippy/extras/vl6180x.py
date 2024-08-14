# VL6180X I2c-based ToF Distance sensor support
# Sections of code taken from Adafruit Library for the VL6180X sensor published under the MIT liscense
# https://github.com/adafruit/Adafruit_CircuitPython_VL6180X/tree/main

import logging
from . import bus


VL6180X_I2C_ADDR = 0x29

VL6180X_MEM_ADDR = {
    "IDENT_MODEL_ID" : 0x000,
    "IDENT_MODEL_REV_MAJOR" : 0x001,
    "IDENT_MODEL_REV_MINOR" : 0x002,
    "IDENT_MODULE_REV_MAJOR" : 0x003,
    "IDENT_MODULE_REV_MINOR" : 0x004,
    "IDENT_DATE_HI" : 0x006,
    "IDENT_DATE_LO" : 0x007,
    "IDENT_TIME" : 0x008,
    "SYS_MODE_GPIO0" : 0x010,
    "SYS_MODE_GPIO1" : 0x011,
    "SYS_HISTORY_CTRL" : 0x012,
    "SYS_INTERRUPT_CONFIG_GPIO" : 0x014,
    "SYS_INTERRUPT_CLEAR" : 0x015,
    "SYS_FRESH_OUT_OF_RESET" : 0x016,
    "SYS_GROUPED_PARAMETER_HOLD" : 0x017,
    "SYSRANGE_START" : 0x018,
    "SYSRANGE_THRESH_HIGH" : 0x019,
    "SYSRANGE_THRESH_LOW" : 0x01A,
    "SYSRANGE_INTERMEASUREMENT_PERIOD" : 0x01B,
    "SYSRANGE_MAX_CONVERGENCE_TIME" : 0x01C,
    "SYSRANGE_CROSSTALK_COMPENSATION_RATE" : 0x01E,
    "SYSRANGE_CROSSTALK_VALID_HEIGHT" : 0x021,
    "SYSRANGE_EARLY_CONVERGENCE_ESTIMATE" : 0x022,
    "SYSRANGE_PART_TO_PART_RANGE_OFFSET" : 0x024,
    "SYSRANGE_RANGE_IGNORE_VALID_HEIGHT" : 0x025,
    "SYSRANGE_RANGE_IGNORE_THRESHOLD" : 0x026,
    "SYSRANGE_MAX_AMBIENT_LEVEL_MULT" : 0x02C,
    "SYSRANGE_RANGE_CHECK_ENABLES" : 0x02D,
    "SYSRANGE_VHV_RECALIBRATE" : 0x02E,
    "SYSRANGE_VHV_REPEAT_RATE" : 0x031,
    "RES_RANGE_STATUS" : 0x04D,
    "RES_INTERRUPT_STATUS_GPIO" : 0x04F,
    "RES_HISTORY_BUFFER_x" : 0x052,
    "RES_RANGE_VAL" : 0x062,
    "RES_RANGE_RAW" : 0x064,
    "RES_RANGE_RETURN_RATE" : 0x066,
    "RES_RANGE_REFERENCE_RATE" : 0x068,
    "RES_RANGE_RETURN_SIGNAL_COUNT" : 0x06C,
    "RES_RANGE_REFERENCE_SIGNAL_COUNT" : 0x070,
    "RES_RANGE_RETURN_AMB_COUNT" : 0x074,
    "RES_RANGE_REFERENCE_AMB_COUNT" : 0x078,
    "RES_RANGE_RETURN_CONV_TIME" : 0x07C,
    "RES_RANGE_REFERENCE_CONV_TIME" : 0x080,
    "READOUT_AVERAGING_SAMPLE_PERIOD" : 0x10A,
    "FIRMWARE_BOOTUP" : 0x119,
    "I2C_SLAVE_DEVICE_ADDRESS" : 0x212,
}

RANGE_ERROR_CODES = [ #Index is the number read from the register
    None, # 0000
    "VCSEL Continuity Test", # 0001
    "VCSEL Watchdog Test", # 0010
    "VCSEL Watchdog", # 0011
    "PLL1 Lock", # 0100
    "PLL2 Lock", # 0101
    "Early Convergence Estimate", # 0110
    "Max Convergence", # 0111
    "No Target Ignore", # 1000
    "Not used", # 1001
    "Not used", # 1010
    "Max Signal To Noise Ratio", # 1011
    "Raw Ranging Algo Underflow", # 1100
    "Raw Ranging Algo Overflow", # 1101
    "Ranging Algo Underflow", # 1110
    "Ranging Algo Overflow" # 1111

]

class VL6180XL:
    def __init__(self,config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, VL6180X_I2C_ADDR, default_speed=100000)
        self.distance = 0
        self.printer.add_object("vl6180x" + self.name, self)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
    
    def handle_connect(self):
        self._init_vl6180x()
        
    
    def _write_1_byte(self, mem_addr:int, value:int):
        """Write 1 byte to 16-bit memory register. 

        Args:
            mem_addr (int): Memory address to write to
            value (int): Value to write
        """
        send = bytes([
            (mem_addr>>8)&0xFF,
            mem_addr&0xFF, 
            value&0xFF
        ])
        self.i2c.i2c_write()
    
    def _read_1_byte(self, mem_addr:int) -> bytearray:
        """Read 1 byte from memory address

        Args:
            mem_addr (int): Memory address to write to

        Returns:
            bytearray: 1 byte value read from sensor
        """
        ret = self.i2c.i2c_read(
            bytes([(mem_addr>>8)&0xFF, mem_addr&0xFF]),
            1
        )
        return bytearray(ret["response"])[0]

    def _read_2_byte(self, mem_addr:int) -> bytearray:
        """Read 2 byte from memory address

        Args:
            mem_addr (int): Memory address to write to

        Returns:
            bytearray: 2 byte value read from sensor
        """
        ret = self.i2c.i2c_read(
            bytes([(mem_addr>>8)&0xFF, mem_addr&0xFF]),
            2
        )
        return bytearray(ret["response"])[:2]
    
    
    def _init_vl6180x(self):
        """Ensure VL6180X is connected and initialize
        """
        # private settings from page 24 of app note
        if self._read_1_byte(VL6180X_MEM_ADDR["IDENT_MODEL_ID"]) != 0xB4:
            logging.exception(f"Connected i2c device doesn't match with VL6180X specifications")
        
        if self._read_1_byte(VL6180X_MEM_ADDR["SYS_FRESH_OUT_OF_RESET"])&0x01 == 1:
            self._load_settings()
            self._write_1_byte(VL6180X_MEM_ADDR["SYS_FRESH_OUT_OF_RESET"], 0x00)


    def _load_settings(self):
        """Load desired settings into the VL6180X
        """
        self._write_8(0x207, 0x01)
        self._write_8(0x208, 0x01)
        self._write_8(0x096, 0x00)
        self._write_8(0x097, 0xFD)
        self._write_8(0x0E3, 0x00)
        self._write_8(0x0E4, 0x04)
        self._write_8(0x0E5, 0x02)
        self._write_8(0x0E6, 0x01)
        self._write_8(0x0E7, 0x03)
        self._write_8(0x0F5, 0x02)
        self._write_8(0x0D9, 0x05)
        self._write_8(0x0DB, 0xCE)
        self._write_8(0x0DC, 0x03)
        self._write_8(0x0DD, 0xF8)
        self._write_8(0x09F, 0x00)
        self._write_8(0x0A3, 0x3C)
        self._write_8(0x0B7, 0x00)
        self._write_8(0x0BB, 0x3C)
        self._write_8(0x0B2, 0x09)
        self._write_8(0x0CA, 0x09)
        self._write_8(0x198, 0x01)
        self._write_8(0x1B0, 0x17)
        self._write_8(0x1AD, 0x00)
        self._write_8(0x0FF, 0x05)
        self._write_8(0x100, 0x05)
        self._write_8(0x199, 0x05)
        self._write_8(0x1A6, 0x1B)
        self._write_8(0x1AC, 0x3E)
        self._write_8(0x1A7, 0x1F)
        self._write_8(0x030, 0x00)
        # Recommended : Public registers - See data sheet for more detail
        self._write_8(VL6180X_MEM_ADDR["SYS_MODE_GPIO1"], 0x10)  # Enables polling for 'New Sample ready'
        # when measurement completes
        self._write_8(VL6180X_MEM_ADDR["READOUT_AVERAGING_SAMPLE_PERIOD"], 0x30)  # Set the averaging sample period
        # (compromise between lower noise and
        # increased execution time)
        self._write_8(0x03F, 0x46)  # Sets the light and dark gain (upper
        # nibble). Dark gain should not be
        # changed.
        self._write_8(VL6180X_MEM_ADDR["SYSRANGE_VHV_REPEAT_RATE"], 0xFF)  # sets the # of range measurements after
        # which auto calibration of system is
        # performed
        self._write_8(0x040, 0x63)  # Set ALS integration time to 100ms
        self._write_8(VL6180X_MEM_ADDR["SYSRANGE_VHV_RECALIBRATE"], 0x01)  # perform a single temperature calibration
        # of the ranging sensor

        # Optional: Public registers - See data sheet for more detail
        self._write_8(VL6180X_MEM_ADDR["SYSRANGE_INTERMEASUREMENT_PERIOD"], 0x09)  # Set default ranging inter-measurement
        # period to 100ms
        self._write_8(0x03E, 0x31)  # Set default ALS inter-measurement period
        # to 500ms
        self._write_8(VL6180X_MEM_ADDR["SYS_INTERRUPT_CONFIG_GPIO"], 0x24)  # Configures interrupt on 'New Sample
        # Ready threshold event'

        self.reactor.pause(self.reactor.monotonic()+0.1) #Pause everything for 100ms to allow the sensor to initialize

    def _read_range_single(self) -> int:
        """Read a single value from the sensor

        Returns:
            int: Distance in mm
        """
        while ((self._read_1_byte(VL6180X_MEM_ADDR["RES_RANGE_STATUS"]) & 0x01) != 1):
            continue
        
        self._write_1_byte(VL6180X_MEM_ADDR["SYSRANGE_START"], 0x01)

        while ((self._read_1_byte(VL6180X_MEM_ADDR["RES_INTERRUPT_STATUS_GPIO"]) & 0x04) != 4):
            continue
        
        range = self._read_1_byte(VL6180X_MEM_ADDR["RES_RANGE_VAL"])

        self._write_1_byte(VL6180X_MEM_ADDR["SYS_INTERRUPT_CLEAR"], 0x07)

        return range
    
    def _verify_range_status(self) -> bool:
        """Verify that the RES_RANGE_STATUS register doesn't store a value indicative of an error

        Returns:
            bool: Returns true if No error, false if there is an error
        """
        status = self._read_1_byte(VL6180X_I2C_ADDR["RES_RANGE_STATUS"])
        if status == 0:
            return True
        else:
            logging.warning(f"VL6180X Range Read Failed. Error Code: {bin(status)[2:]} Meaning: {RANGE_ERROR_CODES[status]}")
            return False

    def _measure_distance(self):
        dist = self._read_range_single
        MAX_RETRIES = 10
        i = 0
        while i < MAX_RETRIES:
            if self._verify_range_status is True:
                self.distance = dist
                return
        
        logging.warning("Max retries failed to get valid distance reading")

    def _sample_vl6180x(self, eventtime):
        self._measure_distance
    
    def get_status(self, eventtime):
        return {
            "distance": self.distance
        }




def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("VL6180X", VL6180XL)