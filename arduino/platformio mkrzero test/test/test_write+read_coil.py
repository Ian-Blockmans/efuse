import serial
from easymodbus import modbusClient

COMPORT = 'COM7'

ser = serial.Serial(COMPORT, 9600, 8, stopbits=2, timeout=1)
ser.close()

modbus_efuse = modbusClient.ModbusClient(COMPORT)
modbus_efuse.connect()

class Test_Modbus:
    def test_write_read_1coil(self):
        modbus_efuse.write_single_coil(1, True)
        coils = modbus_efuse.read_coils(1, 1)
        assert coils[0] == True
    def test_write_read_multiple(self):
        modbus_efuse.write_multiple_coils(0, [True, True])
        coils = modbus_efuse.read_coils(0, 2)
        assert coils == [True,True]



