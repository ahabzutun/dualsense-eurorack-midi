#!/usr/bin/env python3
"""
Tiny mDNS responder for the Percussa SSP: answers "ssp.local" with the SSP's address on whichever
network the question came from (Wi-Fi or Ethernet), so `ssh root@ssp.local` works on any Wi-Fi
without knowing the address DHCP gave it. Only A records for one name, nothing else.

Installed on the SSP as /root/ssp_mdns.py, started by /etc/init.d/S85mdns. Python 3.9, no modules
beyond the standard library.
"""
import fcntl
import socket
import struct
import sys
import time

NAME = (sys.argv[1] if len(sys.argv) > 1 else "ssp") + ".local"
GROUP, PORT = "224.0.0.251", 5353
IFACES = ("wlan0", "eth0")
TTL = 120


def iface_addr(name):
    """(address, netmask) of an interface, or None."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        req = struct.pack("256s", name.encode()[:15])
        addr = fcntl.ioctl(s.fileno(), 0x8915, req)[20:24]        # SIOCGIFADDR
        mask = fcntl.ioctl(s.fileno(), 0x891B, req)[20:24]        # SIOCGIFNETMASK
        return socket.inet_ntoa(addr), socket.inet_ntoa(mask)
    except OSError:
        return None
    finally:
        s.close()


def same_net(a, b, mask):
    ai, bi, mi = (struct.unpack("!I", socket.inet_aton(x))[0] for x in (a, b, mask))
    return ai & mi == bi & mi


def encode_name(name):
    return b"".join(bytes([len(p)]) + p.encode() for p in name.split(".")) + b"\0"


def read_name(data, o):
    labels, jumped, end = [], False, o
    for _ in range(64):
        n = data[o]
        if n & 0xC0 == 0xC0:
            if not jumped:
                end = o + 2
            o = ((n & 0x3F) << 8) | data[o + 1]
            jumped = True
            continue
        if n == 0:
            return ".".join(labels), (end if jumped else o + 1)
        labels.append(data[o + 1:o + 1 + n].decode("latin1"))
        o += 1 + n
    raise ValueError("bad name")


def questions(data):
    """[(name, qtype, unicast?)]"""
    qd = struct.unpack("!H", data[4:6])[0]
    o, out = 12, []
    for _ in range(qd):
        name, o = read_name(data, o)
        qtype, qclass = struct.unpack("!HH", data[o:o + 4])
        o += 4
        out.append((name.lower(), qtype, bool(qclass & 0x8000)))
    return out


def answer(qid, addr, legacy, question):
    flags = 0x8400                                          # response, authoritative
    head = struct.pack("!HHHHHH", qid if legacy else 0, flags, 1 if legacy else 0, 1, 0, 0)
    q = (encode_name(NAME) + struct.pack("!HH", 1, 1)) if legacy else b""
    cls = 1 if legacy else 0x8001                           # cache-flush bit on multicast answers
    rr = encode_name(NAME) + struct.pack("!HHIH", 1, cls, TTL, 4) + socket.inet_aton(addr)
    return head + q + rr


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    sock.bind(("", PORT))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
    sock.settimeout(5.0)
    joined = set()
    last = 0.0
    while True:
        now = time.time()
        if now - last > 10:                                 # Wi-Fi dongle / DHCP may change: re-check
            last = now
            for name in IFACES:
                a = iface_addr(name)
                if a and a[0] not in joined:
                    try:
                        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                        socket.inet_aton(GROUP) + socket.inet_aton(a[0]))
                        joined.add(a[0])
                    except OSError:
                        pass
        try:
            data, (src, sport) = sock.recvfrom(9000)
        except socket.timeout:
            continue
        except OSError:
            time.sleep(1)
            continue
        try:
            if len(data) < 12 or struct.unpack("!H", data[2:4])[0] & 0x8000:
                continue                                    # not a query
            qs = [q for q in questions(data) if q[0] == NAME and q[1] in (1, 255)]
            if not qs:
                continue
            mine = [a for a in (iface_addr(n) for n in IFACES) if a]
            addr = next((a for a, m in mine if same_net(a, src, m)), None)
            if addr is None:
                continue
            legacy = sport != PORT
            qid = struct.unpack("!H", data[0:2])[0]
            pkt = answer(qid, addr, legacy, qs[0])
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(addr))
            if legacy or qs[0][2]:
                sock.sendto(pkt, (src, sport))              # legacy / unicast-response question
            if not legacy:
                sock.sendto(pkt, (GROUP, PORT))
        except Exception:
            continue


if __name__ == "__main__":
    main()
