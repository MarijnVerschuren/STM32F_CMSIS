from struct import pack, unpack
from rich import print

# OLD
#  dev_id: 8
#  info: 4
#  alt: 4
#  port: 4
#  pin: 4

# NEW
#  dev_id: 16
#  alt: 4
#  port: 4
#  pin: 4
#  _ (info): 4

data = [
	0x000000,
	0x640708,
	0x640709,
	0x64070A,
	0x64070B,
	0x64070C,
	0x64070F,
	0x640713,
	0x640716,
	0x640717,
	0x510700,
	0x510701,
	0x510702,
	0x510703,
	0x510704,
	0x510733,
	0x510734,
	0x510735,
	0x510736,
	0x510737,
	0x65080B,
	0x65080C,
	0x650826,
	0x650827,
	0x650828
]
ndata = []

if __name__ == "__main__":
	for dat in data:
		dev_id, ia, pp = unpack("<BBB", dat.to_bytes(3, "big"))
		print(hex(dev_id), hex(ia), hex(pp))
		ap = (ia << 4 | pp >> 4) & 0xFF
		pi = (pp << 4 | ia >> 4) & 0xFF
		ndata.append(f"0x{pack('<HBB', dev_id, ap, pi).hex()}")
	
	print(ndata)