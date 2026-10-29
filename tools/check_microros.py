#!/usr/bin/env python3
"""Check the actual generated library contract before compiling production firmware."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

LIMITS = {
    'RMW_UXRCE_MAX_NODES': 1,
    'RMW_UXRCE_MAX_PUBLISHERS': 16,
    'RMW_UXRCE_MAX_SUBSCRIPTIONS': 4,
    'RMW_UXRCE_MAX_SERVICES': 10,
    'RMW_UXRCE_MAX_CLIENTS': 0,
    'RMW_UXRCE_MAX_HISTORY': 4,
}
MTU = 1200
HEADERS = ('rmw_microxrcedds_c/config.h', 'uxr/client/config.h',
           'rclc_parameter/rclc_parameter.h')
MESSAGES = (('geometry_msgs', 'Twist'), ('sensor_msgs', 'Range'),
            ('sensor_msgs', 'Imu'), ('sensor_msgs', 'BatteryState'),
            ('nav_msgs', 'Odometry'), ('diagnostic_msgs', 'DiagnosticStatus'),
            ('diagnostic_msgs', 'DiagnosticArray'))
SERVICES = (('std_srvs', 'SetBool'), ('std_srvs', 'Trigger'))

def defines(path):
    return dict(re.findall(r'^[ \t]*#define[ \t]+(\w+)[ \t]+([^\n]+)', path.read_text(), re.M))

def check_headers(root):
    errors = []
    try:
        rmw = defines(root / 'include' / HEADERS[0])
        uxr = defines(root / 'include' / HEADERS[1])
        params = defines(root / 'include' / HEADERS[2])
    except OSError as exc:
        return [f'missing generated header: {exc.filename}']
    for key, expected in LIMITS.items():
        if rmw.get(key, '').strip() != str(expected):
            errors.append(f'{key}: expected {expected}, found {rmw.get(key, "missing")}')
    if uxr.get('UXR_CONFIG_CUSTOM_TRANSPORT_MTU', '').strip() != str(MTU):
        errors.append(f'custom MTU must be {MTU}; rebuild with UCLIENT_CUSTOM_TRANSPORT_MTU={MTU}')
    if params.get('RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES', '').strip() != '6':
        errors.append('expected the pinned Jazzy parameter API with six executor handles')
    if not re.search(r'^#define RMW_UXRCE_TRANSPORT_CUSTOM\s*$',
                     (root / 'include' / HEADERS[0]).read_text(), re.M):
        errors.append('the archive must use custom transport')
    return errors

def fingerprints(root):
    paths = ['libmicroros.a', *('include/' + p for p in HEADERS)]
    return {p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in paths}

def check(root, nm, record=False, readelf="arm-none-eabi-readelf"):
    errors = check_headers(root)
    if errors:
        return errors
    lib = root / 'libmicroros.a'
    if not lib.is_file():
        return ['libmicroros.a missing; run make microros']
    try:
        out = subprocess.run([nm, '-g', '--defined-only', str(lib)],
                             check=True, text=True, capture_output=True).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        return [f'cannot inspect ARM archive with {nm}: {exc}']
    symbols = set(re.findall(r'\b[ATDBRWV]\s+(\S+)', out))
    for package, name in MESSAGES:
        symbol = f'rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__{package}__msg__{name}'
        if symbol not in symbols:
            errors.append(f'missing message type support: {package}/{name}')
    for package, name in SERVICES:
        symbol = f'rosidl_typesupport_microxrcedds_c__get_service_type_support_handle__{package}__srv__{name}'
        if symbol not in symbols:
            errors.append(f'missing service type support: {package}/{name}')
    for symbol in ('rclc_parameter_server_init_with_option', 'rmw_uros_sync_session',
                   'rmw_uros_set_custom_transport'):
        if symbol not in symbols:
            errors.append(f'missing required symbol: {symbol}')
    if errors:
        return errors
    try:
        attrs = subprocess.run([readelf, '-A', str(lib)], check=True, text=True, capture_output=True).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        return [f'cannot inspect ARM ABI with {readelf}: {exc}']
    arches = set(re.findall(r'Tag_CPU_arch: ([^\n]+)', attrs))
    if arches != {'v8-M.mainline'} or 'Tag_ABI_VFP_args:' in attrs:
        return ['archive ABI must be ARM v8-M.mainline with base AAPCS/softfp arguments']
    manifest = root / 'contract-sha256.json'
    current = fingerprints(root)
    if record:
        manifest.write_text(json.dumps(current, sort_keys=True, indent=2) + '\n')
    else:
        try:
            expected = json.loads(manifest.read_text())
        except (OSError, ValueError):
            return ['verified build manifest missing; run make microros']
        if current != expected:
            errors.append('archive/generated headers differ from verified build manifest; rebuild')
    return errors

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', type=Path)
    parser.add_argument('--nm', default='arm-none-eabi-nm')
    parser.add_argument('--readelf', default='arm-none-eabi-readelf')
    parser.add_argument('--record', action='store_true', help='seal a newly built candidate after verification')
    args = parser.parse_args()
    errors = check(args.library, args.nm, args.record, args.readelf)
    if errors:
        print('micro-ROS contract failed:\n  ' + '\n  '.join(errors) + '\nRun make microros.', file=sys.stderr)
        return 1
    print('micro-ROS contract: custom MTU1200, 16 pubs / 4 subs / 10 services, Jazzy types verified')
    return 0

if __name__ == '__main__':
    sys.exit(main())
