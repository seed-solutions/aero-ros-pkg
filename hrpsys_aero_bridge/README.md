To use Futaba RSC-U485 with Ubuntu 14.04, Please apply follows:

```
% sudo modprobe ftdi-sio
% sudo sh -c "echo '0115 0008' > /sys/bus/usb-serial/drivers/ftdi_sio/new_id"
```

If it is ok, you can show /dev/ttyUSBn
 (n = 0, 1, ..., will change if you have another USB serial).



To install it to your system permanently

- add following lines to `/etc/rc.local`
```
modprobe ftdi-sio
echo "0115 0008" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```

- (optional) To show device name via lsusb, please modify usb.ids
-- arround L12495 (previous lines: 1113 Medion AG)
```
1115 Futaba
     0008 RSC-U485
```
-- note that emacs will brake character code
