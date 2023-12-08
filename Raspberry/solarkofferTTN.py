import requests
import json

# Replace with your own values
api_key  = 'NNSXS.GCJ6V5CG6QFTVWBICEJPONOVUKT2NTF7H6YS5EQ.ZVGSTDOZH5ICGPGVB4QUYA3BMTVFA3GUFWWDZPS6O2XECSRDB3VA'
app_id = 'solarkoffer'
base_url = 'https://eu1.cloud.thethings.network/api/v3'

def retrieve_data_from_ttn():
    url = f"{base_url}/as/applications/{app_id}/packages/storage/uplink_message"
    headers = {
        'Authorization': f'Bearer {api_key}',
        'Content-Type': 'application/json'
    }

    try:
        response = requests.get(url, headers=headers)
        if response.status_code == 200:
            data = response.json()
            # Process the retrieved data here
            # print(json.dumps(data, indent=2))  # Print the data for demonstration
            
            #data = json.loads(jdata)
            
            # Extract required values
            received_at = data['result']['received_at']
            decoded_payload = data['result']['uplink_message']['decoded_payload']
            analog_in_1 = decoded_payload['analog_in_1']
            analog_in_2 = decoded_payload['analog_in_2']
            analog_in_3 = decoded_payload['analog_in_3']
            analog_in_4 = decoded_payload['analog_in_4']
            analog_in_5 = decoded_payload['analog_in_5']
            
            # Print the extracted values
            print(f"received_at         (UTC): {received_at}")
            print(f"KFZ_Batterie Status   (V): {analog_in_1}")
            print(f"PV Leistung           (W): {analog_in_2}")
            print(f"PV-Leistung max Heute (W): {analog_in_3}")
            print(f"PV Ertrag Gestern    (Wh): {analog_in_4}")
            print(f"ESP LiPo Status       (V): {analog_in_5}")
            
            
            
        else:
            print(f"Failed to retrieve data. Status code: {response.status_code}")
    except requests.RequestException as e:
        print(f"Request failed: {e}")

# Call the function to retrieve data
retrieve_data_from_ttn()