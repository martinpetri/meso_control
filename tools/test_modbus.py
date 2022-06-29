from pymodbus.client.sync import ModbusTcpClient
client = ModbusTcpClient(host='192.168.178.51',port=502)
print("connection: " + str(client))
connection = client.connect()
print("connected: " + str(client.is_socket_open()))
def main():
    x = 0

    while 1:
        
        if client.is_socket_open(): 
            rc = rr = client.read_holding_registers(0, 5, unit=1)
            rr = client.read_holding_registers(32000, 1, unit=1)
            client.write_register(address=32000, value=x)
            print(rc.registers)
            print(rr.registers[0])
            x = x + 1
            if x == 25000:
                x = 1

if __name__ == '__main__':
    main()
