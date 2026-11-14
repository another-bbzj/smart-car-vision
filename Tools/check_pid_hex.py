"""Verify the build reserves both PID sectors and emits no code/data into them."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
stem = 'NEUQ_STM32F407VET6_Opensource_Library'
project = ET.parse(root/'MDK-ARM'/f'{stem}.uvprojx')
for tag in ('IROM', 'OCR_RVCT4'):
    regions = project.findall(f'.//{tag}')
    assert regions, f'{tag} missing'
    for region in regions:
        start = int(region.findtext('StartAddress'), 0)
        size = int(region.findtext('Size'), 0)
        assert start == 0x08000000 and start+size <= 0x08040000, 'PID sectors overlap program region'
base = highest = count = 0
for line in (root/'MDK-ARM'/stem/f'{stem}.hex').read_text().splitlines():
    record = bytes.fromhex(line.removeprefix(':'))
    assert sum(record)%256 == 0 and len(record) == record[0]+5
    length = record[0]
    offset = int.from_bytes(record[1:3], 'big')
    kind = record[3]
    if kind == 4:
        base = int.from_bytes(record[4:6], 'big') << 16
    elif kind == 2:
        base = int.from_bytes(record[4:6], 'big') << 4
    elif kind == 0 and length:
        address = base+offset
        assert 0x08000000 <= address and address+length <= 0x08040000, f'Unexpected HEX data at {address:#x}'
        count += length
        highest = max(highest, address+length-1)
assert count
print(f'PASS firmware {count} bytes, last address {highest:#010x}; PID sectors 0x08040000..0x0807ffff excluded')
