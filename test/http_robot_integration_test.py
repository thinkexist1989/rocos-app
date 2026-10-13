"""Exercise HTTP dispatch and real controller execution using HardwareSim only."""
import json
import math
import pathlib
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

binary = pathlib.Path(sys.argv[1]).resolve()
urdf = pathlib.Path(sys.argv[2]).resolve()
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
base = 'http://127.0.0.1:%d' % port
headers = {'Content-Type': 'application/json',
           'X-Rocos-Control-Token': 'debug-token', 'X-Rocos-Client-Id': 'debug-client'}
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def request(path, body=None, method=None, timeout=8):
    method = method or ('POST' if body is not None else 'GET')
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, headers=headers, method=method)
    with opener.open(req, timeout=timeout) as response:
        return json.load(response)


def ok(path, body=None, method=None):
    result = request(path, body, method)
    assert result['success'], (path, result)
    return result['data']


def state():
    return ok('/api/robot/state')


def positions():
    return [j['position'] for j in state()['joint_states']]


def stopped(timeout=7):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if not state()['motion_busy']:
            return state()
        time.sleep(.02)
    raise AssertionError('Motion did not stop: %s' % state())


with tempfile.TemporaryDirectory(prefix='rocos-http-integration-') as tmp:
    log_path = pathlib.Path(tmp) / 'controller.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([str(binary), '--sim=true', '--grpc=false',
                                    '--urdf=' + str(urdf), '--http_host=127.0.0.1',
                                    '--http_port=' + str(port)], cwd=tmp,
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            end = time.monotonic() + 10
            while True:
                try:
                    if state()['joint_states']:
                        break
                except (OSError, urllib.error.URLError, json.JSONDecodeError):
                    assert process.poll() is None, 'Controller exited during startup'
                    assert time.monotonic() < end, 'Controller did not start'
                    time.sleep(.05)
            n = len(state()['joint_states'])
            assert n == 7
            # Viz applies angles by URDF joint name, not by hardware slave name.
            expected_names = [joint.attrib['name'] for joint in ET.parse(urdf).getroot().findall('joint')
                              if joint.find('hardware') is not None and joint.attrib['type'] != 'fixed']
            assert [joint['name'] for joint in state()['joint_states']] == expected_names
            assert [joint['name'] for joint in ok('/api/robot/info')['joint_infos']] == expected_names
            assert state()['hw_state']['hw_type'] == 1
            assert 'current_cycle_time' in state()['hw_state']
            assert 'calibration_feedback' in state()

            # Work-mode selection is distinct from CSP/CSV/CST drive-mode changes.
            for mode in ['jnt_imp', 'cart_imp', 'position']:
                assert ok('/api/robot/workmode', {'mode': mode})['mode'] == mode
                assert state()['work_mode'] == mode
            invalid = request('/api/robot/workmode', {'mode': 'no-such-mode'})
            assert not invalid['success'] and state()['work_mode'] == 'position'
            servo_mode = request('/api/robot/servo/mode', {'mode': 'CSV'})
            assert not servo_mode['success'] and servo_mode['code'] == 3007
            assert all(j['mode'] == 'CSP' for j in state()['joint_states'])

            # Invalid vectors and bounds do not change targets or start threads.
            assert not request('/api/robot/single_axis/move', {'id': n, 'pos': 0})['success']
            assert not request('/api/robot/single_axis/move', {'id': 0, 'pos': 99})['success']
            assert not request('/api/robot/multi_axis/move', {'target_pos': [0]})['success']
            assert not request('/api/robot/jog/joint', {'joints': [1, 1] + [0] * (n-2)})['success']
            assert not state()['motion_busy']

            # Online axis targets must actually execute, including encoder conversion.
            ok('/api/robot/single_axis/move', {'id': 0, 'pos': .08, 'max_vel': .2, 'max_acc': 1, 'max_jerk': 5})
            stopped()
            assert abs(positions()[0] - .08) < .0001, positions()
            info = ok('/api/robot/info')['joint_infos']
            count = info[0]['cnt_per_unit']
            ok('/api/robot/single_axis/move', {'id': 0, 'pos': .04 * count, 'raw_data': True,
                                             'max_vel': .2 * count, 'max_acc': count, 'max_jerk': 5 * count})
            stopped()
            assert abs(positions()[0] - .04) < .0001
            raw = ok('/api/robot/state?raw_data=true')
            assert raw['raw_data'] and abs(raw['joint_states'][0]['position'] / count - .04) < .0001

            target = [.12, -.15, .08, -.25, .05, .15, 0]
            ok('/api/robot/multi_axis/move', {'target_pos': target, 'max_vel': [.3] * n,
                                             'max_acc': [1] * n, 'max_jerk': [5] * n, 'sync': 'time'})
            stopped()
            assert max(abs(a-b) for a,b in zip(positions(), target)) < .0001
            for sync in ['none', 'phase', 'time']:
                ok('/api/robot/multi_axis/sync', {'sync': sync})

            # A single axis stop halts its online target; unified stop selects Axis.
            ok('/api/robot/single_axis/move', {'id': 0, 'pos': .7, 'max_vel': .15, 'max_acc': 1})
            time.sleep(.15)
            ok('/api/robot/single_axis/stop', {'id': 0})
            stopped()
            assert positions()[0] < .6
            ok('/api/robot/single_axis/move', {'id': 0, 'pos': .7, 'max_vel': .15, 'max_acc': 1})
            time.sleep(.1)
            assert ok('/api/robot/stop', {})['stopped_motion_type'] == 'axis'
            stopped()

            # Offline trajectories honor stop on the servo thread. A synchronous
            # HTTP MoveJ must leave a different HTTP worker free to stop it.
            goal = [.6, -.4, .2, -.55, .2, .4, .1]
            response = []
            def move_sync():
                try:
                    response.append(request('/api/robot/movej', {'joints': goal, 'speed': .15,
                                                                 'acceleration': .6, 'jerk': 3,
                                                                 'asynchronous': False}, timeout=10))
                except Exception as error:
                    response.append(error)
            worker = threading.Thread(target=move_sync)
            worker.start()
            end = time.monotonic() + 5
            while state()['motion_type'] != 'trajectory':
                assert time.monotonic() < end
                time.sleep(.01)
            change = request('/api/robot/workmode', {'mode': 'cart_imp'})
            assert not change['success'] and state()['work_mode'] == 'position'
            assert ok('/api/robot/stop', {})['stopped_motion_type'] == 'trajectory'
            stopped()
            worker.join(5)
            assert not worker.is_alive() and response and isinstance(response[0], dict), response
            assert positions()[0] < .55

            # Jogging is a real movement, with explicit and heartbeat timeout stops.
            before = positions()[0]
            for _ in range(6):
                ok('/api/robot/jog/joint', {'joints': [1] + [0]*(n-1), 'speed': .12, 'timeout': .3})
                time.sleep(.05)
            assert abs(positions()[0] - before) > .001
            assert state()['motion_type'] == 'dragging'
            ok('/api/robot/jog/stop', {})
            stopped()
            after = positions()[0]
            time.sleep(.15)
            assert abs(positions()[0] - after) < .0001
            ok('/api/robot/jog/joint', {'joints': [-1] + [0]*(n-1), 'speed': .1, 'timeout': .05})
            stopped()
            for frame in ['BASE', 'FLANGE', 'TOOL', 'OBJECT']:
                ok('/api/robot/jog/cartesian', {'twist': [0]*6, 'frame': frame})
            ok('/api/robot/dragging', {'flag': 0, 'dir': 'POSITIVE', 'max_speed': .1, 'timeout': .05})
            assert ok('/api/robot/stop', {})['stopped_motion_type'] == 'dragging'
            stopped()
            assert not request('/api/robot/jog/svd', {'dim_speeds': [1, 0]})['success']
            ok('/api/robot/jog/svd', {'dim_speeds': [.02], 'timeout': .05})
            stopped()
            ok('/api/robot/jog/nullspace', {'joints': [1, -1, 0, 0, 0, 0, 0], 'speed': .03, 'timeout': .05})
            stopped()
            # Exercise a real Cartesian direction, not only zero-vector parsing.
            before_flange = state()['flange']['position']
            for _ in range(4):
                ok('/api/robot/jog/cartesian', {'twist': [0, 0, 1, 0, 0, 0],
                                               'frame': 'BASE', 'speed': .02, 'timeout': .3})
                time.sleep(.05)
            ok('/api/robot/jog/stop', {})
            stopped()
            after_flange = state()['flange']['position']
            assert abs(after_flange['z'] - before_flange['z']) > 1e-5

            # Existing calibration input and computation, with insufficient-point checks.
            assert request('/api/calibration/run', {'frame': 'tool'})['code'] == 4001
            def pose(x, y, z=0):
                return {'position': {'x': x, 'y': y, 'z': z},
                        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
            for idx, value in [(7, pose(.1,.2)), (8, pose(.2,.2)), (9, pose(.1,.3))]:
                ok('/api/calibration/pose', {'id': idx, 'pose': value})
            calibration = ok('/api/calibration/run', {'frame': 'object'})
            assert not calibration['error_state']
            result = ok('/api/calibration/result')
            assert abs(result['pose']['position']['x'] - .1) < 1e-9
            assert abs(result['pose']['position']['y'] - .2) < 1e-9

            # Tool calibration uses rotational constraints from poses 1..4,
            # followed by orthogonal orientation points 5 and 6.
            for idx, axis in [(1, 'x'), (2, 'y'), (3, 'z')]:
                value = pose(0, 0)
                value['orientation'][axis] = math.sqrt(.5)
                value['orientation']['w'] = math.sqrt(.5)
                ok('/api/calibration/pose', {'id': idx, 'pose': value})
            for idx, value in [(4, pose(0,0)), (5, pose(.1,0)), (6, pose(0,.1))]:
                ok('/api/calibration/pose', {'id': idx, 'pose': value})
            tool = ok('/api/calibration/run', {'frame': 'tool'})
            assert not tool['error_state'] and abs(tool['pose']['position']['x']) < 1e-9

            # Joint teaching uses the bundled configuration if /opt is absent;
            # switching back stops its worker instead of leaving mode busy.
            for mode in ['jnt_admit_teach', 'ee_admit_teach']:
                reply = request('/api/robot/workmode', {'mode': mode}, timeout=10)
                assert reply['success'], reply
                time.sleep(.05)
                ok('/api/robot/workmode', {'mode': 'position'})
                stopped()
            print('PASS: native axis execution, raw conversion, work modes, trajectory/jog stop, watchdog and calibration')
        except Exception:
            print(log_path.read_text()[-8000:], file=sys.stderr)
            raise
        finally:
            process.terminate()
            try:
                process.wait(5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(5)
