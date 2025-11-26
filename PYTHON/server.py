from modbus.server import ModbusTCPServer

SERVER = ModbusTCPServer('192.168.110.101', 20000)
SERVER.start()