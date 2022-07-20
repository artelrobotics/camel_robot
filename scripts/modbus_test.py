from pymodbus.client.sync import ModbusTcpClient
from time import sleep
import threading


def read_inputs() -> None:
    while True:
        try:
            res = client.read_discrete_inputs(address=0, count=8, unit=0x01)
            if res.isError():
                print(f'res = {res}   res type = {type(res)}')
            else:
                print(res.bits)
                sleep(0.02)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)

if __name__ == "__main__":
    client = ModbusTcpClient(host="192.168.1.110", port=502)
    if client.connect():
        thread = threading.Thread(target=read_inputs)
        thread.start()
        print("connected")
        while True:
            try:
                res = client.write_coils(address=2, values=[True]*8 , unit=0x01)
                sleep(0.05)
                print(res)
                res = client.write_coils(address=2, values=[False]*8, unit=0x01)
                print(res)
                sleep(0.05)
            except KeyboardInterrupt:
                print("keyboard interrupt")
                res = client.write_coils(address=2, values=[False]*8, unit=0x01)
                client.close()
                break
            except Exception as e:
                print(e)
                break
    else:
        print("not connected")