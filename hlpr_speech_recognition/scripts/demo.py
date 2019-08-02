
from record import Recorder
from speech2text import TranscribeClient
import sounddevice as sd
import argparse
import os

DEVICE = 6

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Speech2Text demo')
    parser.add_argument('-l', '--list-devices', action='store_true', help='show list of audio devices and exit')
    parser.add_argument('-d', '--device', type=int, default=DEVICE, help='numeric id of input deviec')

    args = parser.parse_args()
    list_devices = args.list_devices
    device = args.device

    if list_devices:
        print(sd.query_devices())
    else:
        recorder = Recorder(device)
        input('Press Enter to start recording')
        record_file = recorder.record()
        print('Recording...')
        input('Press Enter to stop recording')
        recorder.stop()
        print('Recording finished.')

        transcriber = TranscribeClient()
        results = transcriber.transcribe(record_file)

        print('Transcript: {}'.format(results[0]))
        os.remove(record_file)
