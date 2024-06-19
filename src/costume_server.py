#!/usr/bin/env python3

import rospy
from green_fundamentals.srv import SetVideo, SetVideoResponse
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
from threading import Thread

app = Flask(__name__)
socketio = SocketIO(app)

video_dict = {
    "money" : "https://www.youtube.com/watch?v=HMuYfScGpbE",
    "pickup": "https://www.youtube.com/watch?v=wWf7wD6kHyo",
    "localization": "https://www.youtube.com/watch?v=J3DWAJGaf7o"
}

# Global variable to store the current YouTube video URL
current_video_url = video_dict['money']

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
            document.getElementById('video_frame').src = data.video_url;
        });
    </script>
</head>
<body>
    <h1>Current Video</h1>
    <iframe id="video_frame" width="560" height="315" src="{{ video_url }}" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</body>
</html>
'''

@app.route('/')
def index():
    global current_video_url
    return render_template_string(HTML_TEMPLATE, video_url=current_video_url)

def handle_set_video(req):
    global current_video_url
    if req.video == 1: # Money
        current_video_url = video_dict['money']
    elif req.video == 2: # Pickup
        current_video_url = video_dict['pickup']
    elif req.video == 3: # Localization
        current_video_url = video_dict['localization']
    else:
        return SetVideoResponse(success=False)
    
    # Emit a socket event to update the video on the client side
    socketio.emit('update_video', {'video_url': current_video_url})
    return SetVideoResponse(success=True)

def start_flask_app():
    socketio.run(app, host='0.0.0.0', port=5000)

if __name__ == '__main__':
    rospy.init_node('video_server_node')

    # Start the Flask app in a separate thread
    from threading import Thread
    flask_thread = Thread(target=start_flask_app)
    flask_thread.start()

    # Create a service that changes the video URL
    s = rospy.Service('set_video', SetVideo, handle_set_video)

    rospy.spin()
