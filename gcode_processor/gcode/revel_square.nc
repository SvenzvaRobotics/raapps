%
(1001)
(T1  D=0.0625 CR=0.0984 TAPER=5.5deg - ZMIN=-0.0075 - tapered mill)
G90
G17
G20
G28 G91 Z0
G90

(Trace1)
M9
T1 M6
S12000 M3
G54
M7
G0 X-0.5 Y12
Z0.6
Z0.1975
G1 Z-0.0075 F50
X0.5
Y11
X-0.5
Y12
Z0.1975
G0 Z0.6
M9
G28 G91 Z0
G90
G28 G91 X0 Y0
G90
M30
%