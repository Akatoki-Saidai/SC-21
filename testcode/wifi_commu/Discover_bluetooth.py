import bluetooth

def discover_devices():
    print("Scanning for Bluetooth devices...")
    devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True, lookup_class=False)
    print("Found %d devices" % len(devices))

    for addr, name in devices:
        print("  %s - %s" % (addr, name))

discover_devices()
