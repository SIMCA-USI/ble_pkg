import struct
from std_msgs.msg import String, Float64


class TPMSDevice():
    def __init__(self, id, pub, full=False) -> None:
        self.id = id
        self.__manufacturer_data = None
        self.status = 0
        self.battery = 0
        self.temperature = 0
        self.pressure = 0.
        self.pub = pub
        self.full = full

    def parse_status_byte(self, status):
        return {
            'alarm_zero_pressure': bool(status & 0x80),
            'rotating': bool(status & 0x40),
            'standing_still': bool(status & 0x20),
            'begin_rotating': bool(status & 0x10),
            'decreasing_pressure_below_20.7_psi': bool(status & 0x08),
            'rising_pressure': bool(status & 0x04),
            'decreasing_pressure_above_20.7_psi': bool(status & 0x02),
            'unknown': bool(status & 0x01)
        }

    def parse_tpms_data(self, data):
        if len(data) < 7:
            raise ValueError("Data is too short to parse")
        status, battery, temp, pressure, checksum = struct.unpack('>BBBHH', data)
        status_bits = self.parse_status_byte(status)
        battery_voltage = battery / 10.0
        temperature_c = temp
        pressure_psi = (pressure - 145) / 145
        return status_bits, battery_voltage, temperature_c, pressure_psi

    def parse_full(self, data):
        status_bits = 0
        _, _, _, pressure, temp, battery_voltage, status = struct.unpack('<HHIIIBB', data)
        temperature_c = temp / 100.
        pressure_psi = (pressure / 100000.0) * 1.13 #* 14.5038
        return status_bits, battery_voltage, temperature_c, pressure_psi

    @property
    def manufacturer_data(self):
        return self.__manufacturer_data

    @manufacturer_data.setter
    def manufacturer_data(self, data):
        if data is not None:
            self.__manufacturer_data = data
            try:
                if self.full:
                    self.status, self.battery, self.temperature, self.pressure = self.parse_full(
                        bytearray.fromhex(data))
                else:
                    self.status, self.battery, self.temperature, self.pressure = self.parse_tpms_data(
                        bytearray.fromhex(data))
                self.pub.publish(Float64(data=self.pressure))
            except Exception as e:
                raise Exception(f'Error parsing tpms_data {self.id = } {data} {e}')

    def __repr__(self):
        return f"TPMSDevice(id={self.id}, status={self.status}, battery={self.battery}, temperature={self.temperature}, pressure={self.pressure})"

    def __str__(self):
        return f"TPMSDevice: id={self.id}, status={self.status}, battery={self.battery}, temperature={self.temperature}, pressure={self.pressure}"

    def publicar(self):
        self.pub.publish(Float64(data=self.pressure))
