
import bluetooth

target_name = "My Phone"
target_address = None

nearby_devices = bluetooth.discover_devices()
print (nearby_devices)
