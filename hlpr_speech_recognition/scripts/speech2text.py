from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import sys
import io

class TranscribeClient(object):
    def __init__(self):
        self.client = speech.SpeechClient()
        self.SUPPORTED_FILE_TYPES = ['flac']

    def transcribe(self, filename):
        filetype = filename.split('.')[-1].lower()
        if not filetype in self.SUPPORTED_FILE_TYPES:
            raise NotImplementedError('filetype not supported.')

        with io.open(filename, 'rb') as audio_file:
            content = audio_file.read()
            audio = types.RecognitionAudio(content=content)

        config = types.RecognitionConfig(language_code='en-US')
        response = self.client.recognize(config, audio)

        results = []
        for result in response.results:
            results.append(result.alternatives[0].transcript)

        return results

if __name__ == '__main__':
    filename = sys.argv[1]
    transcriber = TranscribeClient()
    results = transcriber.transcribe(filename)

    for result in results:
        print('Transcript: {}'.format(result))
