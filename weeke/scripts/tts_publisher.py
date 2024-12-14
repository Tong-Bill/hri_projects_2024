#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String
from ros_vosk.msg import speech_recognition
from vosk import Model, KaldiRecognizer
import pyaudio


# This file listens to the user, has TTS speak it back, then publishes the message in text format
# nao_animation file will receive that text, then move robot accordingly
def MySpeech():
	rospy.init_node('microphone_listener', anonymous=True)
	pub = rospy.Publisher('/tts_phrase', String, queue_size=10)

	# Load Vosk model for speech recognition
	# Model path was not working from installed path, moved it to same path as the scripts
	model = Model(model_name="vosk-model-small-en-us-0.15") 
	# How to use kaldiRecognizer: https://snyk.io/advisor/python/vosk/functions/vosk.KaldiRecognizer
	recognizer = KaldiRecognizer(model, 16000)

	# Starts up the input stream for microphone
	# Pyaudio code based on: https://people.csail.mit.edu/hubert/pyaudio/docs/
	p = pyaudio.PyAudio()
	stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
	stream.start_stream()
	rospy.loginfo("Curently listening...")

	while not rospy.is_shutdown():
		data = stream.read(4000, exception_on_overflow=False)
		if len(data) == 0:
		    continue
        
		if recognizer.AcceptWaveform(data):
		    result = recognizer.Result()
		    text = result.split('"')[3]
		    rospy.loginfo(f"Your words: {text}")
		    pub.publish(text)

		    # Use eSpeak to speak back the recognized phrase
		    subprocess.run(['espeak', text])

if __name__ == '__main__':
    try:
        MySpeech()
    except rospy.ROSInterruptException:
        pass
