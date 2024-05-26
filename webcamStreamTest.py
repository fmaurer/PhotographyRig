import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid
import time
import av
from fractions import Fraction
from picamera2 import Picamera2

from aiohttp import web
import aiohttp_cors
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay, MediaStreamTrack

ROOT = os.path.dirname(__file__)

relay = None
webcam = None
cam = Picamera2()
cam.configure(cam.create_video_configuration(main={"size": (1280, 720)}))
cam.start()
pcs = {}

class PiCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.counter = 0

    async def recv(self):
        img = cam.capture_array()
        frame = av.VideoFrame.from_ndarray(img, format='bgr24')
        frame.pts = self.counter
        frame.time_base = Fraction(1, 30)
        self.counter += 1
        return frame

async def webrtc(request):
    params = await request.json()
    if params["type"] == "offer":
        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        pcs[pc_id] = pc

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                pcs.pop(pc_id, None)

        cam_track = PiCameraTrack()
        pc.addTrack(cam_track)
        
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type, "id": pc_id}),
        )

    elif params["type"] == "answer":
        pc = pcs[params["id"]]
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))

        return web.Response(
            content_type="application/json",
            text=json.dumps({}),
        )

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs.values()]
    await asyncio.gather(*coros)
    pcs.clear()

async def index(request):
    with open(os.path.join(ROOT, 'index_rtc.html'), 'r') as f:
        return web.Response(text=f.read(), content_type='text/html')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC camera-streamer")
    parser.add_argument("--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Port for HTTP server (default: 8080)")
    parser.add_argument("--cert_file", help="SSL certificate file (optional)")
    parser.add_argument("--key_file", help="SSL key file (optional)")
    parser.add_argument("--verbose", "-v", action="count")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file and args.key_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_post("/webrtc", webrtc)
    app.router.add_get("/", index)  # Route to serve the index.html file

    
    # Configure CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
            )
    })
    
    for route in list(app.router.routes()):
        cors.add(route)
    
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)
