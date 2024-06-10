import threading
from traceback import format_exc

import rclpy
from bluepy.btle import Scanner, DefaultDelegate
from example_interfaces.msg import String
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Float64
from ble_pkg.utils.tpms_device import TPMSDevice


class BLEScanDelegate(DefaultDelegate):
    def __init__(self, node):
        super().__init__()
        self.node: BLEScannerNode = node

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if (isNewDev or isNewData) and dev.addr in self.node.allowed_devices :
            message = f"Dispositivo: {dev.addr}, RSSI={dev.rssi} dB"
            self.node.get_logger().info(f'{dev.addr =} {dev.getScanData() = }')
            for (adtype, desc, value) in dev.getScanData():
                self.node.get_logger().debug(f'for {adtype = } {desc = } {value = }')
                if desc == "Complete Local Name":
                    message += f", Nombre Completo: {value}"
                if desc == "Short Local Name":
                    message += f", Nombre Corto: {value}"
                if desc == "Manufacturer":
                    message += f", Datos del Fabricante: {value}"
                    self.node.update_device(dev.addr, value)
            # self.node.get_logger().error(f'{tpms_sensor}')
            self.node.get_logger().info(f'{message}')


class BLEScannerNode(Node):
    def __init__(self):
        super().__init__(node_name='tpms_monitor', namespace='tpms_monitor_ns', start_parameter_services=True,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=20))
        self.logger.set_level(self._log_level.value)

        self.device_publisher_fr = self.create_publisher(Float64, 'ble_devices/fr', 10)
        self.device_publisher_rr = self.create_publisher(Float64, 'ble_devices/rr', 10)
        self.device_publisher_fl = self.create_publisher(Float64, 'ble_devices/fl', 10)
        self.device_publisher_rl = self.create_publisher(Float64, 'ble_devices/rl', 10)
        self.allowed_devices = {
            # "38:29:00:00:63:0b": TPMSDevice('rr', self.device_publisher_rr),
            # '38:b4:00:00:36:03': TPMSDevice('fr', self.device_publisher_fr),
            # '38:ad:00:00:4e:5a': TPMSDevice('fl', self.device_publisher_fl),
            # '38:b0:00:00:8b:63': TPMSDevice('rl', self.device_publisher_rl),


            "82:ea:ca:32:ab:b5": TPMSDevice('rr', self.device_publisher_rr, True),
            '81:ea:ca:22:b7:02': TPMSDevice('fr', self.device_publisher_fr, True),
            '80:ea:ca:12:b6:f2': TPMSDevice('fl', self.device_publisher_fl, True),
            '83:ea:ca:42:b5:22': TPMSDevice('rl', self.device_publisher_rl, True),
        }

        self.shutdown = False
        self.status_devices = {}
        self.scanner = Scanner().withDelegate(BLEScanDelegate(self))
        self.scanning_thread = threading.Thread(target=self.scan_loop)
        self.scanning_thread.start()
        self.t_pub = self.create_timer(5, callback=self.callback_timer)

    def update_device(self, dev_addr, manufacturer_data):
        tpms_sensor: TPMSDevice = self.allowed_devices[dev_addr]
        tpms_sensor.manufacturer_data = manufacturer_data
        self.get_logger().error(f'{tpms_sensor}')
        #self.device_publisher.publish(String(data=f'{self.allowed_devices}'))
        self.t_pub.reset()

    def callback_timer(self):
        #self.device_publisher.publish(String(data=f'{self.allowed_devices}'))
        for mac, device in self.allowed_devices.items():
            device:TPMSDevice
            device.publicar()
        pass

    def on_shutdown(self):
        self.shutdown = True

    def scan_loop(self):
        while rclpy.ok() and not self.shutdown:
            self.scanner.scan(5.0)  # Escaneo continuo con bloques de 5 segundos


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = BLEScannerNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Decision: Keyboard interrupt')
        manager.on_shutdown()
    except Exception:
        print(format_exc())
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
