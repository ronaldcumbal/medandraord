import logging
import time
from threading import Thread, Event

from respeaker import Microphone


def task(quit_event):
    mic = Microphone(quit_event=quit_event)

    while not quit_event.is_set():
        if mic.wakeup('respeaker'):
            print('Wake up')
            data = mic.listen()
            text = mic.recognize(data)
            if text:
                print('Recognized %s' % text)


def main():
    logging.basicConfig(level=logging.DEBUG)

    quit_event = Event()
    thread = Thread(target=task, args=(quit_event,))
    thread.daemon = True
    thread.start()
    while True:
        print("HEre")

        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print('Quit')
            quit_event.set()
            break
    time.sleep(1)


def test_record():
    import time
    mic = Microphone()
    mic.record('hello.wav', seconds=3)
    time.sleep(3)
    mic.quit()


if __name__ == '__main__':
    main()
    # test_record()