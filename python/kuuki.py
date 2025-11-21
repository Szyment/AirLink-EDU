#!/usr/bin/env python3
"""Air Quality Logger - Arduino to ThingSpeak"""

import serial
import requests
import sys
import logging
from typing import Optional, Dict

# Config
SERIAL_PORT = '/dev/cu.usbmodem101'
BAUD_RATE = 9600
THINGSPEAK_KEY = '6X1TNE0ORF0JMW0Q'
THINGSPEAK_URL = 'https://api.thingspeak.com/update'

FIELDS = {
    'pm1_0': ('field1', int),
    'pm2_5': ('field2', int),
    'pm10_0': ('field3', int),
    'temp_c': ('field4', float),
    'press_pa': ('field5', float),
    'humidity_pct': ('field6', float),
    'alt_m': ('field7', float)
}

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


def verify_checksum(line: str) -> bool:
    """XOR checksum verification"""
    parts = line.split(',')
    if len(parts) != 8:
        return False
    
    try:
        checksum_rx = int(parts[7], 16)
        data = ','.join(parts[:7])
        checksum_calc = 0
        for c in data:
            if c != ',':
                checksum_calc ^= ord(c)
        return checksum_calc == checksum_rx
    except:
        return False


def parse_line(line: str) -> Optional[Dict]:
    """Parse CSV to dict"""
    parts = line.split(',')
    if len(parts) != 8:
        return None
    
    try:
        return {
            'pm1_0': int(parts[0]),
            'pm2_5': int(parts[1]),
            'pm10_0': int(parts[2]),
            'temp_c': float(parts[3]),
            'press_pa': float(parts[4]),
            'humidity_pct': float(parts[5]),
            'alt_m': float(parts[6])
        }
    except:
        return None


def upload(data: Dict) -> bool:
    """Upload to ThingSpeak"""
    try:
        payload = {'api_key': THINGSPEAK_KEY}
        for field, (ts_field, _) in FIELDS.items():
            if field in data:
                payload[ts_field] = data[field]
        
        r = requests.get(THINGSPEAK_URL, params=payload, timeout=10)
        return r.status_code == 200 and r.text != '0'
    except:
        return False


def main():
    logger.info("Starting Air Quality Logger")
    
    # Find port
    import glob
    ports = glob.glob('/dev/cu.usbmodem*')
    port = ports[0] if ports else SERIAL_PORT
    
    # Connect
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
        logger.info(f"Connected to {port}")
    except serial.SerialException as e:
        logger.error(f"Cannot open port: {e}")
        sys.exit(1)
    
    # Main loop
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            # Verify checksum
            if not verify_checksum(line):
                logger.warning("Checksum failed")
                continue
            
            # Parse
            data = parse_line(line)
            if not data:
                continue
            
            # Log
            logger.info(f"PM1.0={data['pm1_0']} PM2.5={data['pm2_5']} PM10={data['pm10_0']} | "
                       f"T={data['temp_c']:.1f}°C RH={data['humidity_pct']:.0f}% | "
                       f"P={data['press_pa']:.0f}Pa Alt={data['alt_m']:.1f}m")
            
            # Upload
            if upload(data):
                logger.info("Uploaded")
            else:
                logger.warning("Upload failed")
                
    except KeyboardInterrupt:
        logger.info("Stopped")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
