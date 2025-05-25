import os
import cv2
import time
import argparse
from pathlib import Path
from datetime import datetime
from libcamera import controls # type: ignore

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # Handle systems without Picamera2 installed

def parse_args():
    """
    Parses command-line arguments for calibration settings.
    Note: These arguments primarily apply to the PiCamera.
    For optimal results, manual configuration of camera parameters is recommended.
    """
    parser = argparse.ArgumentParser(description="Capture images for camera calibration.")
    parser.add_argument(
        "--calibration_name",
        default=f"camera_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        help="Name of the calibration session."
    )
    parser.add_argument(
        "--img_size",
        nargs=2,
        type=int,
        default=[1536, 864],
        help="Image resolution as two integers: width height."
    )
    parser.add_argument(
        "--delay",
        type=int,
        default=2,
        help="Delay in seconds before starting the camera (PiCamera only)."
    )
    return parser.parse_args()

def main():
    args = parse_args()
    img_width, img_height = args.img_size

    # Define the default save path for calibration images
    base_path = Path(__file__).resolve().parent
    default_save_path = base_path / "calibration_images" / "raw" / args.calibration_name

    # Prompt user to confirm or change the calibration path
    confirm = input(
        f"The current calibration path is set to:\n"
        f"{default_save_path}\n"
        "Would you like to proceed with this path? (yes/no): "
    ).strip().lower()

    if confirm in ['yes', 'y']:
        calibration_path = default_save_path
    elif confirm in ['no', 'n']:
        new_folder = input("Enter the new folder name for saving images: ").strip()
        calibration_path = base_path / "calibration_images" / "raw" / new_folder
    else:
        print("Invalid input. Exiting the script.")
        return

    print(f"Calibration images will be saved to: {calibration_path}")

    # Prompt user to select the camera type
    camera_options = ["USB", "PiCamera"]
    while True:
        camera_choice = input(f"Select the camera type ({'/'.join(camera_options)}): ").strip()
        if camera_choice in camera_options:
            print(f"Camera selected: {camera_choice}")
            break
        print("Invalid choice. Please select either 'USB' or 'PiCamera'.")

    # Create the calibration directory if it doesn't exist
    os.makedirs(calibration_path, exist_ok=True)

    if camera_choice == "USB":
        # Initialize USB camera
        cam = cv2.VideoCapture(0)
        if not cam.isOpened():
            print("Error: Unable to access the USB camera.")
            return

        img_count = 0
        print("Press 's' to save an image, 'Esc' to exit.")
        while True:
            ret, frame = cam.read()
            if not ret:
                print("Failed to capture image from USB camera.")
                break

            cv2.imshow('Calibration Image Capture', frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # Esc key
                print("Exiting image capture.")
                break
            elif key == ord('s'):
                img_filename = calibration_path / f"img_{img_count:03d}.png"
                cv2.imwrite(str(img_filename), frame)
                print(f"Image saved: {img_filename}")
                img_count += 1

        cam.release()
        cv2.destroyAllWindows()

    elif camera_choice == "PiCamera":
        if Picamera2 is None:
            print("Picamera2 library is not installed. Please install it to use the PiCamera.")
            return

        # Initialize PiCamera
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (img_width, img_height), "format": 'XBGR8888'}
        )
        config["controls"] = {"AfMode": 1} 
        picam2.configure(config)
        picam2.start()
        time.sleep(args.delay)  # Allow camera to adjust
        picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

        print("Press 'c' to capture an image, 'q' to quit.")
        while True:
            user_input = input("Enter command: ").strip().lower()
            if user_input == 'c':
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                img_filename = calibration_path / f"image_{timestamp}.jpg"
                picam2.capture_file(str(img_filename))
                print(f"Image captured and saved: {img_filename}")
            elif user_input == 'q':
                print("Exiting image capture.")
                break
            else:
                print("Invalid input. Press 'c' to capture or 'q' to quit.")

        picam2.stop()

if __name__ == "__main__":
    main()
