#!/usr/bin/env python3
"""
Simple HTTP server for the Robotic Arm 3D Controller.
Run this script and open http://localhost:8000 in your browser.
"""

import http.server
import socketserver
import os
import webbrowser
from functools import partial

PORT = 8000

# Change to the script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

Handler = partial(http.server.SimpleHTTPRequestHandler, directory=os.getcwd())

print(f"""
╔══════════════════════════════════════════════════════════╗
║        RoboArm 3D - Servidor de Desarrollo               ║
╠══════════════════════════════════════════════════════════╣
║  Servidor iniciado en: http://localhost:{PORT}             ║
║  Presiona Ctrl+C para detener                            ║
╚══════════════════════════════════════════════════════════╝
""")

try:
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        webbrowser.open(f"http://localhost:{PORT}")
        httpd.serve_forever()
except KeyboardInterrupt:
    print("\nServidor detenido.")
except OSError as e:
    if e.errno == 98:
        print(f"Error: El puerto {PORT} ya está en uso. Intenta cerrar otras aplicaciones o usar otro puerto.")
    else:
        raise
