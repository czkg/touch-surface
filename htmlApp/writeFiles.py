from flask import Flask, render_template
from flask import Response
from flask import request
from flask_cors import CORS, cross_origin
import json
import logging
import os, sys
app = Flask(__name__)
CORS(app)

global gl_experiment_name
filename = ""
count = 0

@app.route('/', methods = ['POST', 'GET'])
def render():
    data = request.form
    key = data["key"]
    x = data["ex"]
    y = data["ey"]
    px = data["px"]
    py = data["py"]
    time = data["time"]
    isRecordLast = data["isRecordLast"]
    finger = data["finger"]
    state = data["state"]

    # if key == "":
    # 	return 'EMPTY'
    global filename
    global count

    if count < 20:
	    if isRecordLast == "false":
	    	#Proposed algo
	    	filename = "./" + gl_experiment_name + "_algo.txt"
	    	file = open(filename, 'w+')
	    	print ("saving it in this file ", filename)
	    	file.write(key + ' ' + time + ' ' + x + ' ' + y + ' ' + finger + ' ' + ' ' + state + '\n')
	    	file.close()

	    	#Naive one
	    	filename = "./" + gl_experiment_name + "_naive.txt"
	    	file = open(filename, 'w+')
	    	print ("saving it in this file ", filename)
	    	file.write(key + ' ' + time + ' ' + px + ' ' + py + ' ' + finger + ' ' + ' ' + state + '\n')
	    	file.close()
	    else:
	    	filename = "./" + gl_experiment_name + "_algo.txt"
	    	file = open(filename, 'a+')
	    	file.write(key + ' ' + time + ' ' + x + ' ' + y + ' ' + finger + ' ' + ' ' + state + '\n')
	    	file.close()

	    	#Naive one
	    	filename = "./" + gl_experiment_name + "_naive.txt"
	    	file = open(filename, 'a+')
	    	file.write(key + ' ' + time + ' ' + px + ' ' + py + ' ' + finger + ' ' + ' ' + state + '\n')
	    	file.close()
	    	
    if count >= 20:
	print ("not writing anymore, 20 entries are done\n")		


    count = count + 1

    return 'OK'

if __name__=='__main__':
	print ("this is the main function")
	global gl_experiment_name
	gl_experiment_name = sys.argv[1]
 	app.run(debug=True)
