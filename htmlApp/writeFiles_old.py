from flask import Flask, render_template
from flask import Response
from flask import request
from flask_cors import CORS, cross_origin
import json
import logging
import os
app = Flask(__name__)
CORS(app)

filename = ""

@app.route('/', methods = ['POST', 'GET'])
def render():
    data = request.form
    key = data["key"]
    x = data["x"]
    y = data["y"]
    time = data["time"]
    isRecordLast = data["isRecordLast"]
    finger = data["finger"]
    state = data["state"]

    # if key == "":
    # 	return 'EMPTY'
    global filename

    if isRecordLast == "false":
    	filename = "./" + time + ".txt"
    	file = open(filename, 'w+')
    	file.write(key + ' ' + time + ' ' + x + ' ' + y + ' ' + finger + ' ' + ' ' + state + '\n')
    	file.close()
    else:
    	file = open(filename, 'a+')
    	file.write(key + ' ' + time + ' ' + x + ' ' + y + ' ' + finger + ' ' + ' ' + state + '\n')
    	file.close()


    return 'OK'

if __name__=='__main__':
    app.run(debug=True)