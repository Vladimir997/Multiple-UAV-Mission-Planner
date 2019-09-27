import os, sys, inspect, ast, time, threading
import http.server, webbrowser
from .websocket_server import WebsocketServer

os.chdir("web")


class CombiServer(WebsocketServer):
    def __init__(self, http_port, websocket_port):
        super(CombiServer, self).__init__(websocket_port)
        self.websocket_thread = threading.Thread(target=self.run_forever)

        # just serve index.html from current working directory
        Handler = http.server.SimpleHTTPRequestHandler
        self.http_port = http_port
        self.httpd = http.server.HTTPServer(("", http_port), Handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever)

    # Called for every client connecting (after handshake)
    @staticmethod
    def new_client(client, server):
        print("New client connected and was given id %d" % client['id'])

    # server.send_message_to_all("Hey all, a new client has joined us")

    @staticmethod
    # Called for every client disconnecting
    def client_left(client, server):
        print("Client(%d) disconnected" % client['id'])

    @staticmethod
    # Called when a client sends a message
    def message_received(client, server, message):
        print("Client(%d) said:" % client['id'])
        try:
            lines = ast.literal_eval(message)
            for line in lines:
                print(line)
        except:
            print("couldn't understand message")
            print(message)

    def __enter__(self):
        self.websocket_thread.start()
        self.http_thread.start()
        webbrowser.open("http://localhost:%i" % self.http_port)  # maybe insert sleep before

    def __exit__(self, *args):
        self.shutdown()
        self.httpd.shutdown()
        self.websocket_thread.join()
        self.http_thread.join()


if __name__ == "__main__":
    input(
        "This file on it's own doesn't do much. The following is only a UI Test and you wont be able to control the drone. (Press Enter to continue.)")

    HTTP_PORT = 8000
    WEBSOCKET_PORT = 9001

    print("serving at ports", HTTP_PORT, WEBSOCKET_PORT)

    server = CombiServer(HTTP_PORT, WEBSOCKET_PORT)

    t0 = time.time()

    try:
        with server:
            while True:
                time.sleep(1)
                position = [0, [37.619, -122.376 + (time.time() - t0) * 0.0001]]
                server.send_message_to_all(repr(position))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    print("done")
