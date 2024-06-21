#!/usr/bin/env python3

import rospy
from green_fundamentals.srv import SetVideo, SetVideoResponse
from flask import Flask, render_template_string, send_from_directory
from flask_socketio import SocketIO, emit
from threading import Thread, Event
import signal
import sys
import os
import random

app = Flask(__name__)
socketio = SocketIO(app)

VIDEO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'videos')

video_dict = {}
for category in os.listdir(VIDEO_DIR):
    category_path = os.path.join(VIDEO_DIR, category)
    video_dict[category] = []
    for video in os.listdir(category_path):
        video_path = os.path.join(category_path, video)
        if os.path.isfile(video_path) and video_path.lower().endswith(('.mp4', '.mkv', '.avi', '.mov', '.flv', '.wmv')):
            video_dict[category].append(video_path)

def pick_random_video(category):
    video_choices = video_dict[category]
    random_video = random.choice(video_choices)
    return random_video

# Global variable to store the current YouTube video URL
current_video = pick_random_video("idle")

# HTML template for the web page
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS Video Server</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.4.1/socket.io.min.js"></script>
    <script type="text/javascript" charset="utf-8">
        var socket = io();
        socket.on('update_video', function(data) {
            console.log("New video received");
            document.getElementById('video_frame').src = "/videos/" + data.video_filename + "?" + new Date().getTime();
        });
        function toggleFullScreen() {
            var videoElement = document.getElementById('video_frame');
            if (!document.fullscreenElement) {
                if (videoElement.requestFullscreen) {
                    videoElement.requestFullscreen();
                } else if (videoElement.mozRequestFullScreen) { // Firefox
                    videoElement.mozRequestFullScreen();
                } else if (videoElement.webkitRequestFullscreen) { // Chrome, Safari and Opera
                    videoElement.webkitRequestFullscreen();
                } else if (videoElement.msRequestFullscreen) { // IE/Edge
                    videoElement.msRequestFullscreen();
                }
            } else {
                if (document.exitFullscreen) {
                    document.exitFullscreen();
                } else if (document.mozCancelFullScreen) { // Firefox
                    document.mozCancelFullScreen();
                } else if (document.webkitExitFullscreen) { // Chrome, Safari and Opera
                    document.webkitExitFullscreen();
                } else if (document.msExitFullscreen) { // IE/Edge
                    document.msExitFullscreen();
                }
            }
        }
    </script>
</head>
<body>
    <h1>Current Video</h1>
    <video id="video_frame" width="560" height="315" controls autoplay loop>
        <source src="{{ video_filename }}" type="video/mp4">
        Your browser does not support the video tag.
    </video>
    <br>
    <button onclick="toggleFullScreen()">Toggle Fullscreen</button>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, video_filename=current_video)

@app.route('/videos/<path:filename>')
def serve_video(filename):
    return send_from_directory(VIDEO_DIR, filename)

def handle_set_video(req):
    global current_video
    if req.state == 0:
        current_video = pick_random_video("driving")
    elif req.state == 1:
        current_video = pick_random_video("idle")
    elif req.state == 2:
        current_video = pick_random_video("localizing")
    elif req.state == 3:
        current_video = pick_random_video("money")
    elif req.state == 4:
        current_video = pick_random_video("pickup")
    else:
        return SetVideoResponse(success=False)
    
    # Emit a socket event to update the video on the client side
    socketio.emit('update_video', {'video_filename': current_video})
    return SetVideoResponse(success=True)

def start_flask_app(stop_event):
    socketio.run(app, host='192.168.0.7', port=8080, debug=True, use_reloader=False)
    stop_event.set()

def signal_handler(sig, frame):
    print('\nShutting down gracefully...')
    rospy.signal_shutdown('Signal received')
    shutdown_event.set()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('video_server_node')

    # Event to signal shutdown
    shutdown_event = Event()

    # Start the Flask app in a separate thread
    flask_thread = Thread(target=start_flask_app, args=(shutdown_event,))
    flask_thread.start()

    # Create a service that changes the video URL
    s = rospy.Service('set_video', SetVideo, handle_set_video)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Wait for Flask to shut down
    shutdown_event.wait()
    print("Flask server has shut down.")
