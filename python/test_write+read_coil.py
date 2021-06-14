import serial
from easymodbus import modbusClient

COMPORT = 'COM7'

ser = serial.Serial(COMPORT, 9600, 8, stopbits=2, timeout=1)
ser.close()

modbus_efuse = modbusClient.ModbusClient(COMPORT)
modbus_efuse.connect()

onoff = input("turn on efuse?(y/n)")
if onoff == 'y':
    modbus_efuse.write_single_coil(0, True)
else:
    modbus_efuse.write_single_coil(0, False)

coils = modbus_efuse.read_coils(1, 1) #(start adress, bits)
if coils == 1:
    print("efuse is now on")
else:
    print("efuse is now off")

modbus_efuse.read_coils()

class Test_Modbus:
    def test_write_read_1coil(self):
        modbus_efuse.write_single_coil(1, True)
        coils = modbus_efuse.read_coils(1, 1)
        assert coils[0] == True
    def test_write_read_multiple(self):
        modbus_efuse.write_multiple_coils(0, [True, True])
        coils = modbus_efuse.read_coils(0, 2)
        assert coils == [True,True]
    def test_efuse_turn_on(self):
        modbus_efuse.write_single_coils(1, True)
        input = modbus_efuse.read_inputregisters(1, 1)
        assert input == True
    def test_efuse_turn_off(self):
        modbus_efuse.write_single_coils(1, False)
        input = modbus_efuse.read_inputregisters(1, 1)
        assert input == False
    def test_LCL2(self):
        modbus_efuse.write_single_coil(2, True)
        modbus_efuse.write_single_coil(3, True)
        input = modbus_efuse.read_coils(1, 5)
        sert input == [False,True,False,False,False]
