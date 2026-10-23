import asyncio, socket, sys, time
SRC='169.254.19.219'; PORT=5555
async def probe(ip, sem):
    async with sem:
        loop=asyncio.get_running_loop()
        s=socket.socket(); s.setblocking(False); s.bind((SRC,0))
        try:
            await asyncio.wait_for(loop.sock_connect(s,(ip,PORT)),0.5)
            return ip
        except Exception: return None
        finally: s.close()
async def main():
    sem=asyncio.Semaphore(1500); t=time.time()
    ips=[f'169.254.{b}.{c}' for b in range(1,255) for c in range(1,255)]
    res=await asyncio.gather(*(probe(ip,sem) for ip in ips))
    hits=[r for r in res if r]
    print('hits',hits,'in',round(time.time()-t,1),'s')
asyncio.run(main())
