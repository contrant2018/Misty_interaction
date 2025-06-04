import requests
from mistyPy.Robot import Robot

# Replace “10.0.0.XXX” with the IP you want to test. Try 10.0.0.150, .151, .152… until one works.
candidate_ip = "10.0.0.151"

try:
    # A short timeout to fail fast if that IP isn’t Misty
    r = requests.get(f"http://{candidate_ip}/api/help", timeout=2)
    if r.status_code == 200:
        print(f"Misty found at {candidate_ip}")
        robot = Robot(candidate_ip)
        print("MistyPy client successfully created.")
    else:
        print(f"{candidate_ip} responded, but HTTP {r.status_code}")
except Exception as e:
    print(f"No response from {candidate_ip}: {e}")