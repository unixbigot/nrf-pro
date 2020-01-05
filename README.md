
# NRF button starter

You will need

* A [J-link programming interface](https://www.aliexpress.com/item/32672270083.html?spm=a2g0s.9042311.0.0.27424c4daotHOw)

* A [generic Bluetooth beacon
  device](https://www.aliexpress.com/item/32862703531.html?spm=a2g0s.9042311.0.0.27424c4d1vkpZp)  ([another seller](https://www.aliexpress.com/item/32885909449.html?spm=a2g0s.8937460.0.0.67322e0ePOZ8d1))
  
* Or, an [nRFPro sensor device](https://www.aliexpress.com/item/32808111676.html?spm=a2g0s.9042311.0.0.27424c4daotHOw)
  
* Pogo pins (or careful soldering) to connect your J-link

* Install [nRF5 arduino board module](https://github.com/sandeepmistry/arduino-nRF5) arduino board module
  
* Install [BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral)
  Arduino library 
  
# Preparation

Follow the nRF5 instructions to install.  When you get to the step
about flashing the softdevice (select softdevice S110, BTE) it will
probably fail to download the soft device, so you need to modify
softdevices.txt in your arduino packages folder.

On Mac it's
~/Library/Arduino15/packages/sandeepmistry/hardware/nRF5/0.6.0/softdevices.txt`.
On linux similar path under `~/.arduino15/packages/`.
	

```
s110.url=https://www.nordicsemi.com/-/media/Software-and-other-downloads/SoftDevices/S110/s110nrf51800.zip
s110.filename=s110_nrf51_8.0.0_softdevice.hex
```

# Talking to the device

I like BluePixel Tech's BLE Scanner app for iphone (and similar for
android) for inspecting BLE devices.

I've written a reference client application in NodeJS for inspecting BLE devices
too: https://github.com/unixbigot/flowerscare-client

# Gotchas

If you have a mac or iphone, and you change the characteristics on
your arduino, you may have to reboot your mac/phone for it to detect
the change.   There's probably a bluetooth cache we could flush
somewhere, or a device ID on the device we could change, to avoid
this.

