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
from botocore.exceptions import BotoCoreError, ClientError

class TextToSpeech():
	def __init__(self, voice='Kimberly'):
		self.tag_follow_indices = {}    
		self.tags = []
		self.client = boto3.client('polly')
		self.voice = voice
		self.actions = []
		self.polly_tags = [ 
			'break',	# adding a pause
			'emphasis', # emphasizing words
			'lang',		# specify another language for specific words
			'mark',		# placing a custom tag in your text
			'p',		# adding a pause between paragraphs
			'phoneme',	# using phonetic pronunciation
			'prosody',	# controlling volume, speaking rate and pitch
			'prosody amazon:max-duration',	# setting a maximum duration for synthesized speech
			's',		# adding a pause between sentences
			'say-as',	# controlling how special types of words are spoken
			'speak',	# identifying SSML-Enhanced text
			'sub',		# pronouncing acronyms and abbreviations
			'w',		# improving pronunciation by specifying parts of speech
			'amazon:auto-breaths',	# adding the sound of breathing
			'amazon:effect',
			#'amazon:effect name=\"drc\"',	#adding dynamic range compression
			#'amazon:effect phonation=\"soft\"',	# speaking softly
			#'amazon:effect vocal-tract-length',	# controlling timbre
			#'amazon:effect name=\"whispered\"'	# whispering
			'amazon:breath'	# breathing
		]
	        

	def remove_tags(self, tagged_text):
		untagged_text = ''                  
		open_brackets_idx = [m.start() for m in re.finditer('<', tagged_text)]
		close_brackets_idx = [m.start() for m in re.finditer('>', tagged_text)]
		assert(len(open_brackets_idx)==len(close_brackets_idx))
		
		open_brackets_idx.sort()
		close_brackets_idx.sort()

		pointer = 0
		for i in range(len(open_brackets_idx)):
			retain = ''
			tag = tagged_text[open_brackets_idx[i]:close_brackets_idx[i]+1]

			# keep the tags from amazon polly as part of untagged_text string
			if any(pt for pt in self.polly_tags if ('<'+pt+' ' in tag) or ('<'+pt+'>' in tag) or ('</'+pt in tag)):
				retain = tag
				# print('retaining: ',retain)
			else:
				self.tags.append(tag)
                                
			if(open_brackets_idx[i]!=0):
				untagged_text += tagged_text[pointer:open_brackets_idx[i]] + retain
         
			else:
				untagged_text += retain 

			pointer = close_brackets_idx[i]+1   
		if(pointer!=len(tagged_text)):
			untagged_text = untagged_text + tagged_text[pointer:]       
	                
		return "<speak>{}</speak>".format(untagged_text)

	def extract_behaviors(self, tagged_text):
		# Remove tags from the input string
		untagged_text = self.remove_tags(tagged_text)

		# Store which word the tags come after
		for tag in list(set(self.tags)):
			tag_idxs = [m.start() for m in re.finditer(tag, tagged_text)] # find all occurances
			for tag_idx in tag_idxs:
				if tag_idx!=0:
					substring = tagged_text[:tag_idx]
					if(substring[-1]==' '):
						substring = substring[:-1]
					untagged_substring = self.remove_tags(substring)
					prev_word_idx = substring.count(' ') + 1
				                
					if tag not in self.tag_follow_indices:
						self.tag_follow_indices[tag] = [prev_word_idx]
					else:
						self.tag_follow_indices[tag].append(prev_word_idx)              
				else:       
					if tag not in self.tag_follow_indices:
						self.tag_follow_indices[tag] = [0]
					else:
						self.tag_follow_indices[tag].append(0)

		print(self.tag_follow_indices)
		for t in self.tag_follow_indices:
			for idx in self.tag_follow_indices[t]:
				args = t.strip("<>").split()
				act = args.pop(0)			
				self.actions.append([idx, act, args])


		# Fetch meta info about speech from AWS using boto3
		try:
			response = self.client.synthesize_speech(
				OutputFormat='json',
				SpeechMarkTypes=['viseme','word'],
				Text=untagged_text,
                                TextType="ssml",
				VoiceId=self.voice)
		except (BotoCoreError, ClientError) as error:
			print(error)
		        response = ""
                        
		# Unpack meta info json to an unsorted list of dictionaries
		s = []
		if "AudioStream" in response:
			with closing(response["AudioStream"]) as stream:
				data = stream.read()

				s = data.split('\n') 
				s = [json.loads(line) for line in s if line != '']
		else:
			print("Could not stream audio")
			return untagged_text, []

		word_times = filter(lambda l: l["type"]=="word", s) # Start edits
		for a in self.actions:
			if a[0] > len(word_times)-1:
				a[0] = s[-1]["time"] / 1000.  # convert ms to seconds
			else:
				a[0] = (word_times[a[0]]["time"]) / 1000.  # convert ms to seconds

		data=[]
		for a in self.actions:
			args = a[2]
			data.append({"start":float(a[0]),
				     "type":a[1],
				     "args":args,
			             "id": a[1]}) # End edits


		visemes = map(lambda l: [l["time"],l["value"]], filter(lambda l: l["type"]=="viseme",s))
		for v in visemes:
			data.append({"start":float(v[0]) / 1000.,  # convert ms to seconds
				     "type":"viseme",
				     "args":"",
				     "id": v[1]}) 

		return untagged_text, data


	def phrase_to_file(self, name, tagged_text, output_dir):

		# Remove tags from the input string
		untagged_text, s = obj.extract_behaviors(tagged_text)

		# Fetch audio for speech from AWS using boto3
		spoken_text = self.client.synthesize_speech(
			OutputFormat='mp3',
			Text=untagged_text,
                        TextType="ssml",
			VoiceId=self.voice)		

		# get absolute dir path
		output_dir_path = os.path.abspath(os.path.expanduser(output_dir))

		b = {
			'text': '"'+untagged_text+'"',
			'file': output_dir_path + '/' + name + '.ogg',
			'behaviors': s
		}

		# Write audio to a file
		with open(b['file'],'wb') as f:
			f.write(spoken_text['AudioStream'].read())
			f.close()

		return b

	def say(self, untagged_text, wait=False, interrupt=True):

                try:
                        response = self.client.synthesize_speech(
			        OutputFormat='ogg_vorbis',
			        Text=untagged_text,
                                TextType="ssml",
			        VoiceId=self.voice)
                except:
                        print "Error synthesizing speech"
                        return 0
                        
		with tempfile.TemporaryFile() as f:
			if "AudioStream" in response:
				with closing(response["AudioStream"]) as stream:
					try:
						f.write(stream.read())
					except IOError as error:
						print(error)
						

			else:
				print("Could not stream audio")
				return

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
	tagged_string = "Hello <wave>! How are <lookat face_loc> you? <mark> hello again! </mark>"
	print(tagged_string)
	obj = TextToSpeech(voice='Kimberly')    
	b = obj.phrase_to_file('out',tagged_string, '../../data/')
	obj.say(b['text'], wait=True, interrupt=False)
