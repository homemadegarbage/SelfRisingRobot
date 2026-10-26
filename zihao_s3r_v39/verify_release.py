# 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
for name,expected in manifest['sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
print('Verified',len(manifest['sha256']),'release files')
