# ---------------- Dependencies required --------------------------------------
from flask import Flask

# ---------------- Application implementation ---------------------------------
app = Flask(__name__)

@app.route('/')
def hello_world():
    """
    Simple display of a web message on screen
    """
    return "Hello from a Docker Swarm"

# ----------------- Main implementation ---------------------------------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)