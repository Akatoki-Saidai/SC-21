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
