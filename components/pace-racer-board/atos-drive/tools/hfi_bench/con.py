import serial, time, sys
sp=serial.Serial('/dev/cu.usbmodem2101',115200,timeout=0.2)
sp.read(65536)
for c in sys.argv[1:]:
    w=1.0
    if '@' in c: c,w=c.rsplit('@',1); w=float(w)
    sp.write((c+'\n').encode()); t=time.time(); b=b''
    while time.time()-t<w: b+=sp.read(8192)
    print(f'>>> {c}\n'+b.decode(errors='replace').strip())
sp.close()
