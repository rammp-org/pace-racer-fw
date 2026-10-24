#!/usr/bin/env python3
"""Minimal raw-socket SCPI helper bound to en7. usage: scpi.py HOST[:PORT] 'CMD' ['CMD'...]"""
import socket, sys, subprocess, re
def en7_ip(prefix):
    out = subprocess.run(['ifconfig','en7'],capture_output=True,text=True).stdout
    ips = re.findall(r'inet (\d+\.\d+\.\d+\.\d+)', out)
    for ip in ips:
        if ip.startswith(prefix): return ip
    return ips[0] if ips else None
def query(host, cmds, port=5555, timeout=2.0):
    s = socket.socket(); s.settimeout(timeout)
    src = en7_ip('192.168.1.' if host.startswith('192.168.1.') else '169.254.')
    if src: s.bind((src,0))
    s.connect((host,port))
    out=[]
    for c in cmds:
        s.sendall((c+'\n').encode())
        if '?' in c:
            try: out.append((c, s.recv(4096).decode(errors='replace').strip()))
            except socket.timeout: out.append((c,'<timeout>'))
        else: out.append((c,''))
    s.close(); return out
if __name__=='__main__':
    hp=sys.argv[1]; host,_,port=hp.partition(':'); port=int(port or 5555)
    for c,r in query(host, sys.argv[2:], port): print(f'{c:40s} -> {r}')
