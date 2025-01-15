import logging
logger = logging.getLogger(__name__)

from modbus_device.io_manager import IOManager

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class ModbusSlaveDevice:
    def __init__(self, config):
        # descriptor/name for external registration and multi slave id-ing
        self.name = config['name']

        # connection settings
        self.address = config['address']
        self.port = config.get('port', int(502))
        self.unit = config.get('unit', 0x01)
        self.timeout = config.get('timeout', 3)

        # make client
        # client is not accessible from outside to guarantee thread safety from
        # IOManager. IOManager does not instantiate the client itself, as addressing
        # should be in the scope of a higher abstraction layer (the "device/slave")
        self.__client = ModbusClient(self.address, port=self.port, timeout=self.timeout)

        # self.io_mgr = IOManager(self.client, {**config, "unit": self.unit}) #PYTHON3
        config['unit'] = self.unit
        self.io_mgr = IOManager(self.__client, config)

        self.inputs = self.io_mgr.inputs
        self.outputs = self.io_mgr.outputs

    def connect(self):
        return self.__client.connect()
    
    def fetch(self):
        self.io_mgr.read()

    def write(self, mapping, value):
        self.io_mgr.write(mapping, value)
    
    def attach_callbacks_discrete(self, handler_function):
        self.io_mgr.attach_callbacks_discrete(handler_function)

    def attach_callbacks_registers(self, handler_function):
        self.io_mgr.attach_callbacks_registers(handler_function)

    def attach_callbacks(self, handler_function):
        self.io_mgr.attach_callbacks(handler_function)



