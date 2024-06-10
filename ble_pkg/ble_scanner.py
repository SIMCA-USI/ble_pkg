import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bluepy.btle import Scanner, DefaultDelegate
import threading

class BLEScanDelegate(DefaultDelegate):
    def __init__(self, node):
        super().__init__()
        self.node: Node = node

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev or isNewData:
            message = f"Dispositivo: {dev.addr}, RSSI={dev.rssi} dB"
            for (adtype, desc, value) in dev.getScanData():
                if desc == "Complete Local Name":
                    message += f", Nombre Completo: {value}"
                if desc == "Short Local Name":
                    message += f", Nombre Corto: {value}"
                if desc == "Manufacturer":
                    message += f", Datos del Fabricante: {value}"
            self.node.get_logger().error(f'{message}')
            self.node.device_publisher.publish(String(data=message))

class BLEScannerNode(Node):
    def __init__(self):
        super().__init__('ble_scanner_node')
        self.device_publisher = self.create_publisher(String, 'ble_devices', 10)
        self.scanner = Scanner().withDelegate(BLEScanDelegate(self))
        self.scanning_thread = threading.Thread(target=self.scan_loop)
        self.scanning_thread.start()

    def scan_loop(self):
        while rclpy.ok():
            self.scanner.scan(5.0)  # Escaneo continuo con bloques de 5 segundos

def main(args=None):
    rclpy.init(args=args)
    ble_scanner_node = BLEScannerNode()
    rclpy.spin(ble_scanner_node)
    ble_scanner_node.scanning_thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()