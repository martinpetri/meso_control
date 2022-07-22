from pymodbus.client.sync import ModbusTcpClient
import time
client = ModbusTcpClient(host='192.168.177.25',port=10502)
print("connection: " + str(client))
connection = client.connect()
print("connected: " + str(client.is_socket_open()))
def main():
    x = 0

    while 1:
        
        if client.is_socket_open(): 
            rr = client.read_holding_registers(32004, 1, unit=1)
            client.write_register(address=32004, value=1)
            print(rr.registers)
        time.sleep(10)

if __name__ == '__main__':
    main()
