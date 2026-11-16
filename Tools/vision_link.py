#!/usr/bin/env python3
"""Orange Pi reference encoder for the F407 observation interface.

No image algorithm is included. Call VisionSender with actual camera results.
Offline test vector: python Tools/vision_link.py --hex track
"""
import argparse
import binascii
import secrets
import struct


def frame(kind, sequence, session, payload=b'', processing_age_ms=0):
    if kind not in (0, 1, 2, 3) or not 0 <= sequence <= 65535 or not 1 <= session <= 0xffffffff:
        raise ValueError('invalid type, sequence or session')
    if not 0 <= processing_age_ms <= 200:
        raise ValueError('invalid age')
    if kind == 3:
        count = (len(payload) - 4) // 4 if len(payload) >= 4 else 0
        if len(payload) < 4 or (len(payload) - 4) % 4 or not 1 <= count <= 16:
            raise ValueError('PATH payload must be 4 + 4N bytes, N in 1..16')
    elif len(payload) != {0: 0, 1: 16, 2: 10}[kind]:
        raise ValueError('invalid payload length')
    if kind == 0 and processing_age_ms:
        raise ValueError('HELLO age must be zero')
    body = struct.pack('>BBHHIH', 1, kind, len(payload), sequence, session, processing_age_ms) + payload
    return b'\xa5\x5a' + body + struct.pack('>H', binascii.crc_hqx(body, 0xffff))


def track_payload(valid_mask=0, confidence=0, lateral_mm=0, heading_mrad=0,
                  curvature_milli=0, element=0, tag_id=255, distance_mm=0):
    values = (valid_mask, confidence, lateral_mm, heading_mrad, curvature_milli, element, tag_id, distance_mm)
    if any(not isinstance(x, int) for x in values):
        raise ValueError('wire fields must be integers')
    if (not 0 <= valid_mask <= 15 or not 0 <= confidence <= 1000
            or not -1000 <= lateral_mm <= 1000 or not -3142 <= heading_mrad <= 3142
            or not -20000 <= curvature_milli <= 20000 or not 0 <= element <= 8
            or tag_id not in (*range(8), 255) or not 0 <= distance_mm <= 65535):
        raise ValueError('track observation outside protocol range')
    return struct.pack('>HHhhhBBHH', *values, 0)


def path_payload(points, flags=0, speed_mmps=0):
    """PATH payload: count, flags, speed, then x/y millimetres (right/forward positive)."""
    if not 1 <= len(points) <= 16 or not 0 <= flags <= 15 or not 0 <= speed_mmps <= 5000:
        raise ValueError('path needs 1..16 points, flags 0..15, speed 0..5000 mm/s')
    body = bytearray(struct.pack('>BBH', len(points), flags, speed_mmps))
    for x, y in points:
        if not isinstance(x, int) or not isinstance(y, int) or not -20000 <= x <= 20000 or not -20000 <= y <= 20000:
            raise ValueError('path points must be integer millimetres within +/-20000')
        body += struct.pack('>hh', x, y)
    return bytes(body)


def tile_payload(index, cells):
    if index not in range(4) or len(cells) != 9 or any(not isinstance(x, int) or not 0 <= x <= 6 for x in cells):
        raise ValueError('tile needs index 0..3 and nine cells 0..6 (zero means blank)')
    return bytes([index, *cells])


class VisionSender:
    def __init__(self, port, session=None):
        self.port = port
        self.session = session if session is not None else (secrets.randbits(32) or 1)
        self.sequence = 0

    def _write(self, data):
        if self.port.write(data) != len(data):
            raise OSError('partial vision frame write')

    def hello(self):
        """Call once with the chassis STOPPED; no ACK exists in this receive-only v1 interface."""
        self._write(frame(0, 0, self.session))

    def _send(self, kind, payload, processing_age_ms):
        sequence = (self.sequence + 1) & 65535
        data = frame(kind, sequence, self.session, payload, processing_age_ms)
        self._write(data)
        self.sequence = sequence

    def send_track(self, processing_age_ms=0, **observation):
        self._send(1, track_payload(**observation), processing_age_ms)

    def send_tile(self, index, cells, processing_age_ms=0):
        self._send(2, tile_payload(index, cells), processing_age_ms)

    def send_path(self, points, flags=0, speed_mmps=0, processing_age_ms=0):
        self._send(3, path_payload(points, flags, speed_mmps), processing_age_ms)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hex', required=True, choices=['hello', 'track', 'tile', 'path'])
    args = parser.parse_args()
    payload = (b'' if args.hex == 'hello' else track_payload() if args.hex == 'track'
               else tile_payload(0, [0] * 9) if args.hex == 'tile'
               else path_payload([(0, 50), (0, 100)]))
    print(frame({'hello': 0, 'track': 1, 'tile': 2, 'path': 3}[args.hex], 0 if args.hex == 'hello' else 1, 123, payload).hex(' ').upper())


if __name__ == '__main__':
    main()
