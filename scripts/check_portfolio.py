"""Check presentation links, original-source hashes and preserved HW3 measurements."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
errors=[];links=0
for file in [ROOT/'README.md',*(ROOT/'docs').glob('*.md'),ROOT/'course_sources/README.md']:
    content=file.read_text(encoding='utf-8')
    for target in re.findall(r'\]\(([^)]+)\)',content)+re.findall(r'src="([^"]+)"',content):
        if '://' in target or target.startswith('#'):continue
        links+=1
        if not (file.parent/target.split('#')[0]).exists():errors.append(f'Missing link: {file}: {target}')
manifest=json.loads((ROOT/'docs/source_manifest.json').read_text())
for record in manifest['files']:
    if hashlib.sha256((ROOT/record['path']).read_bytes()).hexdigest()!=record['sha256']:
        errors.append(f'Original changed: {record["path"]}')
for file in ['results.json','summary.json','cartpole_metrics.csv','config.json']:
    path='docs/results/'+file
    if (ROOT/path).exists():
        result=subprocess.run(['git','show',f'ae7498a:{path}'],cwd=ROOT,capture_output=True)
        if result.returncode==0:
            # Git autocrlf normalizes text; compare semantic newline-normalized content.
            if result.stdout.replace(b'\r\n',b'\n')!=(ROOT/path).read_bytes().replace(b'\r\n',b'\n'):
                errors.append(f'Historical metrics changed: {path}')
tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
for path in tracked:
    if any(part in {'.venv','materials','outputs','__pycache__','.env'} for part in Path(path).parts):
        errors.append(f'Unexpected tracked path: {path}')
    if (ROOT/path).stat().st_size>5_000_000 and path not in {'hw3.ipynb'}:
        errors.append(f'Large tracked file: {path}')
print(f'Checked {links} local links, original source hashes, preserved metrics and tracked paths.')
if errors:raise SystemExit('\n'.join(errors))
print('Portfolio checks passed.')
