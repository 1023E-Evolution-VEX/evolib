import csv
import argparse
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MOTORS = {5: 'left_front', 13: 'left_middle', 12: 'left_back',
          17: 'right_front', 14: 'right_middle', 15: 'right_back',
          19: 'intake_bottom', 20: 'intake_top'}
DS = {'type': 'yesoreyeram-infinity-datasource', 'uid': 'evo-sd'}


def panel(number, title, x, ys, position, unit='', description=''):
    fields = [x, *ys]
    return {
        'id': number, 'title': title, 'description': description, 'type': 'xychart',
        'pluginVersion': '13.2.1',
        'datasource': DS, 'gridPos': dict(zip(('x', 'y', 'w', 'h'), position)),
        'targets': [{'refId': 'A', 'datasource': DS, 'type': 'csv', 'source': 'url',
                     'url': 'http://csv/${run:percentencode}', 'url_options': {'method': 'GET'},
                     'parser': 'backend', 'format': 'table', 'root_selector': '',
                     'columns': [{'selector': f, 'text': f, 'type': 'number'} for f in fields],
                     'csv_options': {'skip_empty_lines': True, 'skip_lines_with_error': True}}],
        'fieldConfig': {'defaults': {'unit': unit, 'custom': {'show': 'lines', 'lineWidth': 2,
                                                           'pointSize': {'fixed': 3}, 'axisLabel': unit}},
                        'overrides': [{'matcher': {'id': 'byName', 'options': x},
                                       'properties': [{'id': 'unit', 'value': 's' if x == 'elapsed_s' else 'lengthin'},
                                                      {'id': 'custom.axisLabel', 'value': x}]}]},
        'options': {'mapping': 'auto', 'series': [
            {'x': {'matcher': {'id': 'byName', 'options': x}}}] if ys else [],
            'legend': {'showLegend': True, 'displayMode': 'list', 'placement': 'bottom'},
            'tooltip': {'mode': 'single'}}}


def generate(source=None):
    prefixes = [f'm{port}_{name}' for port, name in MOTORS.items()]
    selected_run = 'example.csv'
    if source is not None:
        with source.open(newline='') as file:
            fields = csv.DictReader(file).fieldnames or []
        required = {'elapsed_s', 'x_in', 'y_in', 'heading_deg', 'linear_speed_ips', 'angular_speed_dps',
                    'battery_mv', 'missed_samples'}
        if not required <= set(fields):
            raise ValueError('CSV is missing required EvoLib telemetry columns')
        prefixes = [f[:-4] for f in fields if re.fullmatch(r'm\d+_\w+_rpm', f) and f[:-4] + '_temp_c' in fields]
        for prefix in prefixes:
            if not all(f'{prefix}_{suffix}' in fields for suffix in ('rpm', 'target_rpm', 'temp_c', 'voltage_mv', 'current_ma')):
                raise ValueError(f'Incomplete motor columns: {prefix}')
        selected_run = source.name
    metrics = lambda suffix: [f'{prefix}_{suffix}' for prefix in prefixes]
    panels = [
        panel(1, 'Executed route (inches)', 'x_in', ['y_in'], (0, 0, 12, 12), 'lengthin',
              'Recorded odometry, not ground truth. Heading zero is +Y, clockwise positive. '
              'Field center is (0, 0); boundaries are +/-72 inches. Pose resets can cause jumps.'),
        panel(2, 'Motor temperature', 'elapsed_s', metrics('temp_c'), (12, 0, 12, 6), 'celsius'),
        panel(3, 'Measured motor speed', 'elapsed_s', metrics('rpm'), (12, 6, 12, 6), 'rotrpm'),
        panel(4, 'Motor current', 'elapsed_s', metrics('current_ma'), (0, 12, 12, 7), 'mamp'),
        panel(5, 'Applied motor voltage', 'elapsed_s', metrics('voltage_mv'), (12, 12, 12, 7), 'mvolt'),
        panel(6, 'Heading', 'elapsed_s', ['heading_deg'], (0, 19, 12, 6), 'degree'),
        panel(7, 'Ground speed magnitude', 'elapsed_s', ['linear_speed_ips'], (12, 19, 12, 6), 'suffix:in/s'),
        panel(8, 'Battery voltage', 'elapsed_s', ['battery_mv'], (0, 25, 12, 6), 'mvolt'),
        panel(9, 'Missed samples', 'elapsed_s', ['missed_samples'], (12, 25, 12, 6), 'short'),
        panel(10, 'Requested motor velocity', 'elapsed_s', metrics('target_rpm'), (0, 31, 12, 7), 'rotrpm',
              'PROS target velocity; voltage-mode move() commands are best compared using the applied voltage panel.'),
        panel(11, 'Angular speed', 'elapsed_s', ['angular_speed_dps'], (12, 31, 12, 7), 'suffix:deg/s')]
    panels[0]['fieldConfig']['defaults'].update(min=-72, max=72)
    panels[0]['fieldConfig']['defaults']['custom']['axisLabel'] = 'Y (inches)'
    panels[0]['fieldConfig']['overrides'][0]['properties'] = [
        {'id': 'unit', 'value': 'lengthin'}, {'id': 'custom.axisLabel', 'value': 'X (inches)'},
        {'id': 'min', 'value': -72}, {'id': 'max', 'value': 72}]
    dashboard = {'uid': 'evo-telemetry', 'title': 'EvoLib autonomous telemetry', 'schemaVersion': 41,
                 'version': 1, 'editable': True, 'tags': ['VEX', 'EvoLib'],
                 'panels': [p for p in panels if p['options']['series']],
                 'time': {'from': 'now-1h', 'to': 'now'}, 'timezone': 'browser',
                 'templating': {'list': [{'name': 'run', 'label': 'CSV filename', 'type': 'textbox',
                                          'query': selected_run, 'current': {'text': selected_run, 'value': selected_run}}]}}
    (ROOT / 'dashboards').mkdir(exist_ok=True)
    (ROOT / 'dashboards/telemetry.json').write_text(json.dumps(dashboard, indent=2) + '\n')
    if source is not None:
        return
    write_sample(ROOT / 'data/example.csv', 'SYNTHETIC_EXAMPLE')


def write_sample(path, label):
    if path.exists():
        with path.open(newline='') as file:
            if any(not row.get('run', '').startswith('SYNTHETIC_') for row in csv.DictReader(file)):
                raise ValueError(f'Refusing to replace a real recording: {path}')
    base = ['sample', 'uptime_ms', 'elapsed_ms', 'elapsed_s', 'run', 'mode', 'x_in', 'y_in',
            'heading_deg', 'linear_speed_ips', 'angular_speed_dps', 'battery_mv', 'battery_capacity_pct', 'missed_samples']
    names = [f'm{port}_{name}_{metric}' for port, name in MOTORS.items()
             for metric in ('rpm', 'target_rpm', 'temp_c', 'voltage_mv', 'current_ma')]
    # Demonstration geometry: 3.25-inch wheels, 1:1 drive, 12-inch track, blue cartridges.
    previous = (-42, -54, math.degrees(math.atan2(84 + 32 * math.pi, 108)))
    previous_rpm = dict.fromkeys(MOTORS, 0.0)
    heat = {p: (46.9 if name.startswith('intake') else 33.5 + .08 * (n % 3))
            for n, (p, name) in enumerate(MOTORS.items())}
    with path.open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(base + names)
        for i in range(301):
            t = i * .05
            phase = t / 15
            u = phase ** 3 * (10 - 15 * phase + 6 * phase ** 2)
            x = -42 + 84 * u + 16 * math.sin(2 * math.pi * u)
            y = -54 + 108 * u
            heading = math.degrees(math.atan2(84 + 32 * math.pi * math.cos(2 * math.pi * u), 108))
            dx, dy, dh = x - previous[0], y - previous[1], heading - previous[2]
            speed, omega = math.hypot(dx, dy) / .05, dh / .05
            mid = math.radians((heading + previous[2]) / 2)
            forward = (dx * math.sin(mid) + dy * math.cos(mid)) / .05
            motors = []
            total_current = 0
            for port, name in MOTORS.items():
                intake = name.startswith('intake')
                load = math.exp(-.5 * ((t - 8.4) / .45) ** 2) if intake else 0
                if intake:
                    ramp = min(1, max(0, (t - .5) / 1.2), max(0, (15 - t) / 1.2))
                    demand = 480 * ramp * ramp * (3 - 2 * ramp)
                    rpm = previous_rpm[port] + (demand - 100 * load - previous_rpm[port]) * .15
                else:
                    side = 1 if name.startswith('left') else -1
                    rpm = (forward + side * math.radians(omega) * 6) * 60 / (math.pi * 3.25)
                    demand = rpm
                if not intake:
                    rpm *= 1 + .001 * math.sin(t * 2 + port)
                acceleration = abs(rpm - previous_rpm[port]) / .05
                current = min(2500, round(65 + abs(rpm) * 1.4 + acceleration * .25 + 650 * load))
                voltage = round(max(-12000, min(12000, demand / 600 * 12000)))
                heat[port] += .05 * (current / 1000) ** 2 * .12
                temperature = 5 * round(heat[port] / 5)
                motors += [round(rpm, 2), 0, temperature, voltage, current]
                previous_rpm[port] = rpm
                total_current += current
            battery = round(12800 - t * 2 - total_current * .045)
            values = [i, 12000 + i * 50, i * 50, round(t, 2), label, 'autonomous',
                      round(x, 4), round(y, 4), round(heading, 3),
                      round(speed, 4), round(omega, 4), battery, 95, 0, *motors]
            writer.writerow(values)
            previous = (x, y, heading)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=Path, help='Generate the dashboard for the motors in a recorded CSV')
    args = parser.parse_args()
    generate(args.csv)
