import threading
import serial
import glob
import time

class StrobeDevice:
    """
    Python interface to your strobe via serial port, with explicit bitmap support.
    Added writeToFile and playStrobeFile methods for chunk streaming.
    """

    BAUDRATE = 250000
    TIMEOUT = 0.1

    # LED index mapping:
    LED_MAP = {
        'central': 0,
        'north_outer': 1, 'north_inner': 2,
        'east_outer': 3,  'east_inner': 4,
        'south_outer': 5, 'south_inner': 6,
        'west_outer': 7,  'west_inner': 8
    }
    RING_LEDS = [idx for name, idx in LED_MAP.items() if name != 'central']

    def __init__(self, port=None, log=False):
        self.log = log
        self._stop_event = threading.Event()
        self._resp_lock = threading.Lock()
        self._responses = []
        self._reader_thread = None

        # auto-detect if no port given
        self.port = port or self._find_port()
        if not self.port:
            raise RuntimeError("No strobe found on any COM port")

        self.ser = serial.Serial(self.port, self.BAUDRATE, timeout=self.TIMEOUT)
        self._start_reader()

    @staticmethod
    def _find_port():
        for p in glob.glob('COM*'):
            try:
                s = serial.Serial(p, StrobeDevice.BAUDRATE, timeout=0.5)
                s.write(b'0;')
                resp = s.readline().decode(errors='ignore')
                s.close()
                if resp.startswith('0'):
                    return p
            except:
                pass
        return None

    def _start_reader(self):
        def _reader():
            while not self._stop_event.is_set():
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    cmd_char = line[0]
                    payload = line[2:] if len(line) > 2 else ''
                    with self._resp_lock:
                        self._responses.append((cmd_char, payload))
                    if self.log:
                        print(f"RX<{cmd_char}>: {payload}")
            if self.log:
                print("Reader thread exiting")

        self._reader_thread = threading.Thread(target=_reader, daemon=True)
        self._reader_thread.start()

    def send_command(self, cmd_char, params=""):
        msg = f"{cmd_char}{params};".encode()
        self.ser.write(msg)
        if self.log:
            print(f"TX>{cmd_char}: {params}")

    def get_response(self, want_chars=None, timeout=1.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._resp_lock:
                for i, (c, p) in enumerate(self._responses):
                    if want_chars is None or c in want_chars:
                        return self._responses.pop(i)
            time.sleep(0.01)
        return None

    def writeToFile(self, filename, data_bytes):
        """
        Write raw strobe chunk data to device storage.
        data_bytes: a bytes-like object length divisible by 6.
        """
        # send write command: W<size> filename
        size = len(data_bytes)
        # build param: filename,size
        params = f"{filename},{size}"
        self.send_command('W', params)
        resp = self.get_response(want_chars=['W', 'E'], timeout=5.0)
        if resp and resp[0] == 'W':
            # now send raw bytes
            self.ser.write(data_bytes)
            # wait for Done
            self.get_response(want_chars=['W'], timeout=10.0)
        else:
            raise RuntimeError(f"WriteToFile failed: {resp}")

    def playStrobeFile(self, filename, wait_s=0):
        """
        Play a previously written strobe file by name.
        wait_s: seconds to wait for end (0 = fire and return immediately).
        """
        params = f"{filename},0"
        self.send_command('5', params)
        # initial open-file response
        self.get_response(want_chars=['5','E'], timeout=5.0)
        if wait_s > 0:
            # wait for Done
            self.get_response(want_chars=['5'], timeout=wait_s)

    def play_phase_locked(self, rate_hz, brightness=255, leds=None):
        central_brightness = 0
        leds = leds or self.RING_LEDS
        bitmap = sum(1 << idx for idx in leds)
        params = f"{central_brightness},{bitmap},{brightness},{rate_hz}"
        self.send_command('6', params)
        return self.get_response(want_chars=['6', 'E'], timeout=1.0)

    def cancel_strobe(self):
        self.send_command('C')
    
    def close(self):
        self.cancel_strobe()
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        self.ser.close()
