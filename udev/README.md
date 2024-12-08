# udev rules
This allows us to set a unique identifier for a type of device whenever it's plugged in, which allows for differentiating between the Arduino and the RoboClaw.

[99-wall-e.rules](99-wall-e.rules) is an example that worked for me to name the Arduino as `/dev/arduino` and the RoboClaw as `/dev/roboclaw`. Simply copy this into `/etc/udev/rules.d/` on the Raspberry Pi.

If this doesn't work for you, follow the below steps to set up your own udev rules.

# Creation

1. Plug in a device.

2. Figure out which device you just plugged in (by checking `ls /dev/tty*` for new entries).

3. For the new device (ex `/dev/ttyUSB0`) run the following:
```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk
```

4. Starting from the top of the output, find the first entries for `ATTRS{idVendor}=="XXXX"` and `ATTRS{idProduct}=="YYYY"` and note the values for `XXXX` and `YYYY`

5. Create a new file in `/etc/udev/rules.d/` named something like `99-wall-e.rules`

6. Add the following line, with your own custom `device_name`, using the `XXXX` and `YYYY` values from step 4:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="device_name"
```

7. Repeat these steps for every device of interest. You can add the rules to the same file.
