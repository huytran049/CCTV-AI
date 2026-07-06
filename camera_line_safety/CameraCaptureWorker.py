import threading
import time
import logging
class FreshestFrame(threading.Thread):
    def __init__(self, capture, name='FreshestFrame', sleep_interval=0.01):
        super().__init__(name=name)
        self.capture = capture
        assert self.capture.isOpened(), "Capture device not opened."

        self.cond = threading.Condition()
        self.running = False
        self.frame = None
        self.latestnum = 0
        self.callback = None
        self.sleep_interval = sleep_interval

        self.start()

    def start(self):
        self.running = True
        super().start()

    def release(self, timeout=None):
        self.running = False
        self.join(timeout=timeout)
        self.capture.release()

    def run(self):
        try:
            # time.sleep(5)
            counter = 0
            while self.running:
                # block for fresh frame
                (rv, img) = self.capture.read()
                if not rv: 
                    with self.cond: # lock the condition for this operation
                        self.latestnum = -1
                        self.frame = None
                        self.cond.notify_all()
                else:
                    counter += 1 
                    # publish the frame
                    with self.cond: # lock the condition for this operation
                        self.frame = img if rv else None
                        self.latestnum = counter
                        self.cond.notify_all()
                time.sleep(self.sleep_interval)  # Add sleep to reduce CPU consumption
                if self.callback:
                    self.callback(img)
        except Exception as e:
            print(f"Error in capture thread: {e}")
            logging.error('Error at %s', 'CameraCaptureWorker', exc_info=e)
        
    def read(self, wait=True, seqnumber=None, timeout=None):
        # with no arguments (wait=True), it always blocks for a fresh frame
        # with wait=False it returns the current frame immediately (polling)
        # with a seqnumber, it blocks until that frame is available (or no wait at all)
        # with timeout argument, may return an earlier frame;
        #   may even be (0,None) if nothing received yet

        with self.cond:
            if wait:
                if seqnumber is None:
                    seqnumber = self.latestnum+1
                if seqnumber < 1:
                    seqnumber = 1
                
                rv = self.cond.wait_for(lambda: self.latestnum >= seqnumber, timeout=timeout)
                if not rv:
                    return (self.latestnum, self.frame)

            return (self.latestnum, self.frame)