from pathlib import Path
from picamera2 import Picamera2
import time
import argparse
import logging

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", default="picamera_capture_test.jpg")
    parser.add_argument("--delay", type=int, default=2)
    return parser.parse_args()

def capture_image(save_file: Path, delay: int = 2):
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration())

    try:
        picam2.start()
        time.sleep(delay)
        logging.info(f"Saving image to: {save_file}")
        picam2.capture_file(str(save_file))
    finally:
        picam2.stop()

def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO)

    save_path = Path(__file__).resolve().parent / "test_images" / "pi_camera"
    save_path.mkdir(parents=True, exist_ok=True)

    save_file = save_path / args.filename
    capture_image(save_file, delay=args.delay)

if __name__ == "__main__":
    main()
