import struct
x = 173.125
s = struct.pack('>f', x)
''.join('%2.2x' % ord(c) for c in s)

i = struct.unpack('>l', s)[0]
print hex(i)
0x432d2000