#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fastapi import FastAPI
import uvicorn
import threading

class ToolServer(Node):
    def __init__(self):
        super().__init__('tool_server')
        self.app = FastAPI()

        @self.app.get("/")
        async def root():
            return {"message": "Tool Server is running"}

        # Start FastAPI server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.start()

    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8080)

def main(args=None):
    rclpy.init(args=args)
    tool_server = ToolServer()
    rclpy.spin(tool_server)
    tool_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()