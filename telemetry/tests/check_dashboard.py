import csv
import json
import math
import importlib.util
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1] / 'grafana'
dashboard = json.loads((root / 'dashboards/telemetry.json').read_text())
with (root / 'data/example.csv').open(newline='') as file:
    reader = csv.DictReader(file)
    fields = reader.fieldnames
    rows = list(reader)
assert len(rows) == 301 and len(fields) == len(set(fields)) == 54
assert all(row['run'] == 'SYNTHETIC_EXAMPLE' for row in rows)
assert all(None not in row for row in rows)
for row in rows:
    assert all(-72 <= float(row[f]) <= 72 for f in ('x_in', 'y_in'))
    for f in fields:
        value = float(row[f]) if f.startswith('m') and f != 'mode' else None
        if f.endswith('_temp_c'):
            assert 25 <= value <= 55 and value % 5 == 0
        if f.endswith('_current_ma'):
            assert value.is_integer() and 0 <= value <= 2500
        if f.endswith('_voltage_mv'):
            assert value.is_integer() and abs(value) <= 12000
        if f.endswith('_rpm'):
            assert abs(value) <= 600
for a, b in zip(rows, rows[1:]):
    distance = math.hypot(float(b['x_in']) - float(a['x_in']), float(b['y_in']) - float(a['y_in']))
    assert abs(distance / .05 - float(b['linear_speed_ips'])) < .003
for a, b, c in zip(rows, rows[1:], rows[2:]):
    dx = float(c['x_in']) - float(a['x_in'])
    dy = float(c['y_in']) - float(a['y_in'])
    if math.hypot(dx, dy) > .02:
        assert abs(math.degrees(math.atan2(dx, dy)) - float(b['heading_deg'])) < .3
route = dashboard['panels'][0]['fieldConfig']
assert route['defaults']['min'] == -72 and route['defaults']['max'] == 72
x_settings = {p['id']: p['value'] for p in route['overrides'][0]['properties']}
assert x_settings['min'] == -72 and x_settings['max'] == 72
for panel in dashboard['panels']:
    assert panel['type'] == 'xychart'
    # Missing versions trigger Grafana's pre-11.1 migration and double-wrap matchers.
    assert tuple(map(int, panel['pluginVersion'].split('.')[:2])) >= (11, 1)
    target = panel['targets'][0]
    assert target['datasource']['uid'] == 'evo-sd'
    assert target['url'] == 'http://csv/${run:percentencode}'
    selected = {column['selector'] for column in target['columns']}
    assert selected <= set(fields)
    assert panel['options']['mapping'] == 'auto'
    assert panel['options']['series'] == [
        {'x': {'matcher': {'id': 'byName', 'options': target['columns'][0]['selector']}}}]
    assert len(selected) > 1
    assert all(column['type'] == 'number' for column in target['columns'])
    assert all(math.isfinite(float(row[field])) for row in rows for field in selected)
assert len(dashboard['panels']) == 11
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('dashboard_generator', root / 'generate.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
test_root = root.parents[1] / 'bin/grafana-tests'
test_root.mkdir(parents=True, exist_ok=True)
source = test_root / 'custom.csv'
mapping = {f: f for f in fields[:14]}
mapping.update({f.replace('m5_left_front', 'm7_arm'): f for f in fields if f.startswith('m5_left_front_')})
with source.open('w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=mapping)
    writer.writeheader()
    writer.writerow({new: rows[0][old] for new, old in mapping.items()})
original = source.read_bytes()
generator.ROOT = test_root
generator.generate(source)
custom = json.loads((test_root / 'dashboards/telemetry.json').read_text())
assert custom['templating']['list'][0]['current']['value'] == 'custom.csv'
assert source.read_bytes() == original
for panel in custom['panels']:
    assert {c['selector'] for c in panel['targets'][0]['columns']} <= set(mapping)
    assert panel['options']['mapping'] == 'auto'
    assert all('frame' not in s and 'y' not in s for s in panel['options']['series'])
assert 'm7_arm_rpm' in json.dumps(custom)
assert 'm5_left_front' not in json.dumps(custom)
print('PASS: example schema, XY mappings, custom motor discovery, and recorded CSV preservation')
