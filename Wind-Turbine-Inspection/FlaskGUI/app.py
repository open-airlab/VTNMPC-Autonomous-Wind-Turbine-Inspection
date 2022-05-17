import cv2
import numpy as np
from flask import Flask, url_for, render_template, Response, request, redirect
from blueprints.inspection.inspection_page import inspection_page_bp
from backend.input import RenderVideoInput

app = Flask(__name__)
app.register_blueprint(inspection_page_bp, url_prefix="/inspection")

video_input = RenderVideoInput(camera_index=0, sleep_time=0, async_stream=False, render=True, verbose=True)


@app.route('/')
def home_page():
    return render_template('index.html')


@app.route('/about')
def about_page():
    return render_template("about.html")


# We need to have all the variables enclosed with brackets <> but the : could be anything
@app.route('/dynamic_var/<dyn_var>:<dyn_var2>')
def dynamic_variable_example(dyn_var, dyn_var2):
    return f'This is a dynamic variable given through the url: {dyn_var} and {dyn_var2}'


@app.route('/video_feed')
def video_feed():
    return Response(video_input.genFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/toggle_feed', methods=["POST"])
def toggle_feed():
    # video_input.render = not video_input.render
    print(f"TOGGLED to {video_input.render}")
    distance = float(request.form["test"])
    return ('', 204)


if __name__ == '__main__':
    app.run(debug=True)
