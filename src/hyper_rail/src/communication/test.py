from communication.camera_executor import Micasense, DFK33GP006
from communication.constants import DEFAULT_CAMERA_HOST, IMAGE_PATH

if __name__ == "__main__":
    Cam = DFK33GP006(IMAGE_PATH, 1, 1)
    captured = Cam.capture_image()
    if captured:
            Cam.transfer_to_local_storage()
            print("Images captured successfully")
            print("camera success")
    else:
        print("Error: Image not capture")
        print("camera fail")