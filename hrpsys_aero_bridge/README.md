To use Futaba RSC-U485 with Ubuntu, Please apply follows:


## 14.04

```
% sudo modprobe ftdi-sio
% sudo sh -c "echo '1115 0008' > /sys/bus/usb-serial/drivers/ftdi_sio/new_id"
```

## 12.04

```
% sudo modprobe ftdi_sio vendor=0x1115 product=0x0008
```


If it is ok, you can show /dev/ttyUSBn
 (n = 0, 1, ..., will change if you have another USB serial).



To install it to your system permanently

- add following lines to `/etc/rc.local`
-- 14.04
```
modprobe ftdi-sio
echo "0115 0008" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```

-- 12.04
```
modprobe ftdi_sio vendor=0x1115 product=0x0008
```

- (optional) To show device name via lsusb, please modify usb.ids
-- arround L12495 (previous lines: 1113 Medion AG)
```
1115 Futaba
     0008 RSC-U485
```
-- note that emacs will brake character code
