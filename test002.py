from opcua import ua, Client
import time
import logging
import sys
a=True
#logging.basicConfig(level=logging.INFO)
#_logger = logging.getLogger('opcua')

if __name__ == "__main__":
    client = Client("opc.tcp://192.168.1.10:4840")
    client.set_user('admin')
    client.set_password('d0a904e3')
    client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
    try:
        client.connect()
        root = client.get_root_node()
        #_logger.info('Objects node is: %r', root)
        objects = client.get_objects_node()
        node2 = client.get_node("ns=5;s=Arp.Plc.Eclr/V3")
        node2.set_value(ua.DataValue(True))
    except:
        pass
    client.disconnect()

