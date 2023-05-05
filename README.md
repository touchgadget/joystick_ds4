# Sony Dual Shock 4 Gamepad

Demo Arduino program that reads axes and buttons over USB and prints them on
the serial console. It works with the Adafruit Feather RP2040 with USB Type A
Host board.

Sample output. The first line is a hex dump of the binary HID report.
The second line decodes the binary data to useful values.

```
DS4 report(64): 017F85818108004800005C350F0100F8FFFEFF70013C21FB0100000000001B0000012C820FD43080000000008000000080000000008000000080000000008000
Count:18,LX:127,LY:133,RX:129,RY:129,dpad:8,L2:0,R2:0,
```

Additional information to decode the values.
```
N=North, S=South, W=West, E=East

LX : (W)0..127..255(E)
LY : (N)0..127..255(S)
RX : (W)0..127..255(E)
RY : (N)0..127..255(S)
dpad : 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW,8=center
L2 : 0..255
R2 : 0..255
```
