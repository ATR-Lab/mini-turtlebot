
import socket
from pynput import keyboard

# --- UDP Setup ---
ESP_IP = "10.177.231.163"      # ← Replace with your ESP32's IP address
UDP_PORT = 5601
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(msg):
    sock.sendto(msg.encode(), (ESP_IP, UDP_PORT))
    print(f"Sent: {msg}")

pressed_keys = set()

def on_press(key):
    if key == keyboard.Key.left and 'left' not in pressed_keys:
        pressed_keys.add('left')
        send_cmd("L1")
    elif key == keyboard.Key.right and 'right' not in pressed_keys:
        pressed_keys.add('right')
        send_cmd("R1")

def on_release(key):
    if key == keyboard.Key.left:
        pressed_keys.discard('left')
        send_cmd("L0")
    elif key == keyboard.Key.right:
        pressed_keys.discard('right')
        send_cmd("R0")
    elif key == keyboard.Key.esc:
        # ESC exits the program
        return False

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("Use ← and → arrows to control LEDs. Press ESC to exit.")
    listener.join()
