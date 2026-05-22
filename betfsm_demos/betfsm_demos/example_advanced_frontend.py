from http.server import SimpleHTTPRequestHandler, HTTPServer

PORT = 8002

class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.path = "example_advanced.html"
        return SimpleHTTPRequestHandler.do_GET(self)

server = HTTPServer(("0.0.0.0", PORT), Handler)
print(f"Serving on http://localhost:{PORT}")
server.serve_forever()
