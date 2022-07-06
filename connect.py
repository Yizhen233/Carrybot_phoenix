from opcua import ua, Client
import time
import logging
import sys

#logging.basicConfig(level=logging.INFO)
# = logging.getLogger('opcua')

if __name__ == "__main__":
    client = Client("opc.tcp://192.168.1.10:4840")
    client.set_user('admin')
    client.set_password('d0a904e3')
    client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
    try:
        client.connect()
        root = client.get_root_node()
        node = client.get_node("ns=5;s=Arp.Plc.Eclr/TEST00")
        node.set_value(ua.DataValue(False))
        print(node.get_value())
        node = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V1")
        node.set_value(ua.DataValue(False))
        print(node.get_value())
        node = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V2")
        node.set_value(ua.DataValue(False))
        print(node.get_value())
        node = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V3")
        node.set_value(ua.DataValue(False))
        print(node.get_value())
        node = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V4")
        node.set_value(ua.DataValue(False))
        print(node.get_value())

    except:
        pass
    client.disconnect()

