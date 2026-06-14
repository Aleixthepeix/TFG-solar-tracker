#!/usr/bin/env python3
"""
test_stream.py — Comprueba si un endpoint de stream HTTP responde.

Uso:
    python test_stream.py <URL>
    python test_stream.py <IP> [puerto]     # asume /stream como path

Ejemplos:
    python test_stream.py http://192.168.0.109:81/stream
    python test_stream.py http://192.168.0.230:4747/video
    python test_stream.py 192.168.0.109 81
"""

import sys
import socket
import urllib.request
import urllib.error
from urllib.parse import urlparse

def test_stream(url: str, timeout: float = 5.0):
    parsed = urlparse(url)
    host = parsed.hostname
    port = parsed.port or 80

    print(f'\nProbando conexión a: {url}')
    print(f'Timeout: {timeout} s\n')

    # ── 1. Alcanzabilidad TCP ────────────────────────────────────────
    print('1. TCP connect... ', end='', flush=True)
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        print('OK')
    except socket.timeout:
        print('TIMEOUT — host inalcanzable (IP incorrecta o red inaccesible)')
        return
    except ConnectionRefusedError:
        print('RECHAZADA — el servidor no está escuchando en ese puerto')
        return
    except OSError as e:
        print(f'ERROR de red: {e}')
        return

    # ── 2. HTTP GET ──────────────────────────────────────────────────
    print('2. HTTP GET... ', end='', flush=True)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            code  = resp.getcode()
            ctype = resp.headers.get('Content-Type', '(no Content-Type)')
            print(f'HTTP {code}')
            print(f'   Content-Type : {ctype}')

            if code == 200 and 'multipart' in ctype:
                print('\n✓ Stream MJPEG activo y respondiendo.')
            elif code == 200:
                print(f'\n⚠ HTTP 200 pero Content-Type inesperado: {ctype}')
                print('  Puede que la URL apunte a una página HTML, no al stream.')
            else:
                print(f'\n✗ HTTP {code}')

    except urllib.error.HTTPError as e:
        print(f'HTTP {e.code} {e.reason}')
        print('  El servidor existe pero ese endpoint no es válido.')
        print('  Prueba con otra ruta (/video, /mjpeg, /stream, etc.)')
    except urllib.error.URLError as e:
        reason = e.reason
        if isinstance(reason, socket.timeout):
            print('TIMEOUT al leer la respuesta HTTP')
        else:
            print(f'ERROR: {reason}')
    except Exception as e:
        print(f'ERROR inesperado: {e}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    arg = sys.argv[1]
    if arg.startswith('http'):
        url = arg
    else:
        port = int(sys.argv[2]) if len(sys.argv) > 2 else 81
        url  = f'http://{arg}:{port}/stream'

    test_stream(url)
