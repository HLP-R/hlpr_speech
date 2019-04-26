#!/usr/bin/env python
import boto3
import pygame
from pygame import mixer
import sys
import re
import json
import tempfile
import os
from contextlib import closing
import io

class TextToSpeech():
	def __init__(self, voice='Kimberly'):
		self.tag_follow_indices = {}    
		self.tags = []
		self.client = boto3.client('polly')
		self.voice = voice
	
	def collect_tags(self, tagged_text):
		self.tags = re.findall(r'<.+?>',tagged_text)

	def remove_tags(self, tagged_text):
		untagged_text = ''                  
		open_brackets_idx = [m.start() for m in re.finditer('<', tagged_text)]
		close_brackets_idx = [m.start() for m in re.finditer('>', tagged_text)]
		assert(len(open_brackets_idx)==len(close_brackets_idx))
		
		open_brackets_idx.sort()
		close_brackets_idx.sort()

		pointer = 0
		for i in range(len(open_brackets_idx)):
			if(open_brackets_idx[i]!=0):
				untagged_text = untagged_text + tagged_text[pointer:open_brackets_idx[i]-1]
			pointer = close_brackets_idx[i]+1   
		if(pointer!=len(tagged_text)):
			untagged_text = untagged_text + tagged_text[pointer:]       
	
		return untagged_text

	def extract_behaviors(self, tagged_text):
		# Remove tags from the input string
		untagged_text = self.remove_tags(tagged_text)
		untagged_word_list = untagged_text.split()
		self.collect_tags(tagged_text)

		# Store which word the tags come after
		for tag in list(set(self.tags)):
			tag_idxs = [m.start() for m in re.finditer(tag, tagged_text)] # find all occurances
			for tag_idx in tag_idxs:
				if tag_idx!=0:
					substring = tagged_text[:tag_idx]
					if(substring[-1]==' '):
						substring = substring[:-1]
					untagged_substring = self.remove_tags(substring)
					prev_word_idx = untagged_substring.count(' ') + 1
				
					if tag not in self.tag_follow_indices:
						self.tag_follow_indices[tag] = [prev_word_idx]
					else:
						self.tag_follow_indices[tag].append(prev_word_idx)              
				else:       
					if tag not in self.tag_follow_indices:
						self.tag_follow_indices[tag] = [0]
					else:
						self.tag_follow_indices[tag].append(0)

		# Fetch meta info about speech from AWS using boto3
		response = self.client.synthesize_speech(
				OutputFormat='json',
				SpeechMarkTypes=['viseme','word'],
				Text=untagged_text,
				VoiceId=self.voice)
		
		# Unpack meta info json to an unsorted list of dictionaries
		s = []
		if "AudioStream" in response:
			with closing(response["AudioStream"]) as stream:
				   data = stream.read()

				   s = data.split('\n') 
				   s = [json.loads(line) for line in s if line != '']
		else:
			print("Could not stream audio")
			

		return untagged_text, s

	def phrase_to_file(self, name, tagged_text, output_dir):

		# Remove tags from the input string
		untagged_text, s = obj.extract_behaviors(tagged_text)

		# Fetch audio for speech from AWS using boto3
		spoken_text = self.client.synthesize_speech(
						OutputFormat='mp3',
						Text=untagged_text,
						VoiceId=self.voice)

		# Write audio to a file
		with open(output_file_loc,'wb') as f:
			f.write(spoken_text['AudioStream'].read())
			f.close()

		# get absolute dir path
		output_dir_path = os.path.abspath(os.path.expanduser(output_dir))

		b = {
		'text': '"'+untagged_text+'"',
		'file': output_dir_path + '/' + name + '.ogg',
		'behaviors': s
		}

		return b

	def say(self, untagged_text, wait=False, interrupt=True):

		response = self.client.synthesize_speech(
				OutputFormat='ogg_vorbis',
				Text=untagged_text,
				VoiceId=self.voice)

		with tempfile.TemporaryFile() as f:
			if "AudioStream" in response:
				with closing(response["AudioStream"]) as stream:
					try:
						f.write(stream.read())
					except IOError as error:
						print(error)
						sys.exit(-1)

			else:
				print("Could not stream audio")
				sys.exit(-1)

			f.seek(0)

			if not pygame.mixer.get_init():
				pygame.mixer.init()
			else:
				if interrupt:
					pygame.mixer.stop()
			channel = pygame.mixer.Channel(5)
			channel.set_volume(1.0)
			sound = pygame.mixer.Sound(f)
			channel.play(sound)

			if wait:
				while channel.get_busy():
					pass
				return -1


		return sound.get_length()

	def is_speaking(self):
		# Returns whether or not audio is being played
        	return not pygame.mixer.get_init() or pygame.mixer.Channel(5).get_busy()

        def shutup(self):
        	if pygame.mixer.get_init():
			pygame.mixer.stop()

if __name__ == '__main__':
	tagged_string = "Hello <wave>! How are <lookat face_loc> you?"
	print(tagged_string)
	obj = TextToSpeech(voice='Kimberly')    
	b = obj.phrase_to_file('out',tagged_string, 'data/')
	obj.say(b['text'], wait=True, interrupt=False)
