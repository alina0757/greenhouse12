import requests

url = "http://esp32_ip_address/api"

# Пример POST-запроса для команды "sensors"
payload = {
  "command": "sensors"
}

response = requests.post(url, data=payload)
if response.status_code == 200:
    print("POST request successful")
    print("Response:", response.text)
else:
    print("POST request failed")

# Пример POST-запроса для команды "pumpon"
payload = {
  "command": "pumpon"
}

response = requests.post(url, data=payload)
if response.status_code == 200:
    print("POST request successful")
    print("Response:", response.text)
else:
    print("POST request failed")