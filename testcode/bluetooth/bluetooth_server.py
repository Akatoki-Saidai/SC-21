import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
 
import array
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
import sys


 
mainloop = None
 
BLUEZ_SERVICE_NAME = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE =      'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE =    'org.freedesktop.DBus.Properties'
 
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE =    'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE =    'org.bluez.GattDescriptor1'
 
class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'
 
class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'
 
class NotPermittedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotPermitted'
 
class InvalidValueLengthException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.InvalidValueLength'
 
class FailedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.Failed'
 
 
class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        self.add_service(TestService(bus, 0))
 
    def get_path(self):
        return dbus.ObjectPath(self.path)
 
    def add_service(self, service):
        self.services.append(service)
 
    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        print('GetManagedObjects')
 
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()
 
        return response
 
 
class Service(dbus.service.Object):
    """
    org.bluez.GattService1 interface implementation
    """
    PATH_BASE = '/org/bluez/example/service'
 
    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)
 
    def get_properties(self):
        return {
                GATT_SERVICE_IFACE: {
                        'UUID': self.uuid,
                        'Primary': self.primary,
                        'Characteristics': dbus.Array(
                                self.get_characteristic_paths(),
                                signature='o')
                }
        }
 
    def get_path(self):
        return dbus.ObjectPath(self.path)
 
    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)
 
    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result
 
    def get_characteristics(self):
        return self.characteristics
 
    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()
 
        return self.get_properties()[GATT_SERVICE_IFACE]
 
 
class Characteristic(dbus.service.Object):
    """
    org.bluez.GattCharacteristic1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)
 
    def get_properties(self):
        return {
                GATT_CHRC_IFACE: {
                        'Service': self.service.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                        'Descriptors': dbus.Array(
                                self.get_descriptor_paths(),
                                signature='o')
                }
        }
 
    def get_path(self):
        return dbus.ObjectPath(self.path)
 
    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)
 
    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result
 
    def get_descriptors(self):
        return self.descriptors
 
    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
 
        return self.get_properties()[GATT_CHRC_IFACE]
 
    @dbus.service.method(GATT_CHRC_IFACE,
                        in_signature='a{sv}',
                        out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise NotSupportedException()
 
    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise NotSupportedException()
 
    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        print('Default StartNotify called, returning error')
        raise NotSupportedException()
 
    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        print('Default StopNotify called, returning error')
        raise NotSupportedException()
 
    @dbus.service.signal(DBUS_PROP_IFACE,
                         signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass
 
class TestService(Service):
    TEST_SVC_UUID = '5A2B2149-66DA-B159-EF44-35E31B1BC15F'
 
    def __init__(self, bus, index):
        Service.__init__(self, bus, index, self.TEST_SVC_UUID, True)
        self.add_characteristic(TestCharacteristic(bus, 0, self))
        self.add_characteristic(TestCharacteristic1(bus, 1, self))
        self.add_characteristic(TestCharacteristic2(bus, 2, self))
 
class TestCharacteristic(Characteristic):
    """
    Dummy test characteristic. Allows writing arbitrary bytes to its value, and
    contains "extended properties", as well as a test descriptor.
    """

    TEST_CHRC_UUID = '943B36FB-863B-2597-9B83-B68F6558C906'
 
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                self.TEST_CHRC_UUID,
                ['read'],
                service)
        self.value = [5]
 
    def ReadValue(self, options):
        self.value = ["Hello, bluetooth"]
        print('TestCharacteristic Read: ' + repr(self.value))
        return self.value
 
class TestCharacteristic1(Characteristic):
    TEST_CHRC_UUID = '12345678-1234-5678-1234-56789abcdef7'
 
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                self.TEST_CHRC_UUID,
                ['read'],
                service)
        self.value = [50]
 
    def ReadValue(self, options):
        self.value = ["hi, this is not used"]
        print('TestCharacteristic Read: ' + repr(self.value))
        return self.value
 
class TestCharacteristic2(Characteristic):
    TEST_CHRC_UUID = '12345678-1234-5678-1234-56789abcdef8'
 
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                self.TEST_CHRC_UUID,
                ['read'],
                service)
        self.value = [55]
 
    def ReadValue(self, options):
        self.value = ["hi, this also is not used"]
        print('TestCharacteristic Read: ' + repr(self.value))
        return self.value
 
def register_app_cb():
    print('GATT application registered')
 
 
def register_app_error_cb(error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()
 
 
def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()
 
    for o, props in objects.items():
        if GATT_MANAGER_IFACE in props.keys():
            return o
 
    return None
 
def main():
    global mainloop
 
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
 
    bus = dbus.SystemBus()
 
    adapter = find_adapter(bus)
    if not adapter:
        print('GattManager1 interface not found')
        return
 
    service_manager = dbus.Interface(
            bus.get_object(BLUEZ_SERVICE_NAME, adapter),
            GATT_MANAGER_IFACE)
 
    app = Application(bus)
 
    mainloop = GObject.MainLoop()
 
    print('Registering GATT application...')
 
    service_manager.RegisterApplication(app.get_path(), {},
                                    reply_handler=register_app_cb,
                                    error_handler=register_app_error_cb)
 
    mainloop.run()
 
if __name__ == '__main__':
    main()





"""
import dbus


# Create a UUID object
service_uuid = "5A2B2149-66DA-B159-EF44-35E31B1BC15F"
characteristic_uuid = "943B36FB-863B-2597-9B83-B68F6558C906"

data = "hello, bluetooth"

def write_to_characteristic(service_uuid, characteristic_uuid, hex_string, bus_name='org.bluez', device_path='/org/bluez/hci0/dev_FF_FF_99_96_64_60'):
    
    # システムバスに接続
    system_bus = dbus.SystemBus()
    
    # UUIDからハイフンを除去
    service_uuid = service_uuid.replace('-', '')
    characteristic_uuid = characteristic_uuid.replace('-', '')
    
    # デバイスオブジェクトを取得
    device = system_bus.get_object(bus_name, device_path)
    
    # サービスUUIDとキャラクタリスティックUUIDを使ってキャラクタリスティックのパスを作成
    characteristic_path = f'/org/bluez/hci0/service{service_uuid}/char{characteristic_uuid}'
    
    # キャラクタリスティックオブジェクトを取得
    characteristic = system_bus.get_object(bus_name, characteristic_path)
    characteristic_interface = dbus.Interface(characteristic, dbus_interface='org.bluez.GattCharacteristic1')
    
    # キャラクタリスティックに値を書き込む
    byte_data = data.encode('utf-8')
    characteristic_interface.WriteValue(byte_data, signature='ay')


write_to_characteristic(service_uuid, characteristic_uuid, data)
"""