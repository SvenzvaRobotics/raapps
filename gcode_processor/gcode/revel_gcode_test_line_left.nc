%
(1001)
(T1  D=0.0625 CR=0.0984 TAPER=5.5deg - ZMIN=0 - tapered mill)
G90
G17
G20
G28 G91 Z0
G90

(line_left)
M9
T1 M6
S12000 M3
G54
M7
G0 X-7 Y15
Z0.602
Z0.2
G1 Z0 F50
Y12
Z0.2
G0 Z0.602
M9
G28 G91 Z0
G90
G28 G91 X0 Y0
G90
M30
%