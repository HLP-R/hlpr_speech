from threading import Thread
import sounddevice as sd
import soundfile as sf
import tempfile
import argparse
import queue
import sys

DEVICE = 6

class Recorder(object):
    def __init__(self, device, type='flac', save_dir=''):
        self.device = device
        self.device_info = sd.query_devices(self.device, 'input')
        self.sample_rate = int(self.device_info['default_samplerate'])
        self.type = type
        self.save_dir = save_dir
        self.recording = False

    def _record(self, filename):
        q = queue.Queue()
        def callback(indata, frames, time, status):
            #if status:
            #    print(status, file=sys.stderr)
            q.put(indata.copy())

        with sf.SoundFile(filename, mode='x', samplerate=self.sample_rate, channels=1) as f:
            with sd.InputStream(samplerate=self.sample_rate, device=self.device,
                                channels=1, callback=callback):
                while self.recording:
                    f.write(q.get())

    def record(self, filename=None):
        if filename is None:
            filename = tempfile.mktemp(prefix='rec_', suffix='.'+self.type, dir=self.save_dir)

        self.recording = True
        record_thread = Thread(target=self._record, args=(filename,))
        record_thread.start()

        return repr(filename)

    def stop(self):
        self.recording = False

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Record sound from microphone')
    parser.add_argument('-l', '--list-devices', action='store_true', help='show list of audio decives and exit')
    parser.add_argument('-d', '--device', type=int, default=DEVICE, help='numeric id of input device')

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
        print('Recording finished: ', record_file)
