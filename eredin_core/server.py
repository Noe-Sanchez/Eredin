import flask

app = flask.Flask(__name__)

# Print JSON PUT request 
@app.route('/put/', methods=['POST'])
def put():
    data = flask.request.json
    print(data)
    return '', 204

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=19000)


