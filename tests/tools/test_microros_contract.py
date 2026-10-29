"""Exercise stale/missing/mismatched generated artifact rejection."""
import sys
sys.dont_write_bytecode = True
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location('contract', Path(__file__).parents[2] / 'tools/check_microros.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.write(contract.HEADERS[0], '#define RMW_UXRCE_TRANSPORT_CUSTOM\n' + '\n'.join(
            f'#define {key} {value}' for key, value in contract.LIMITS.items()))
        self.write(contract.HEADERS[1], '#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU 1200\n')
        self.write(contract.HEADERS[2], '#define RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES 6\n')
        (self.root / 'libmicroros.a').write_bytes(b'synthetic archive for hash tests')
        syms = [f'rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__{p}__msg__{n}'
                for p, n in contract.MESSAGES]
        syms += [f'rosidl_typesupport_microxrcedds_c__get_service_type_support_handle__{p}__srv__{n}'
                 for p, n in contract.SERVICES]
        syms += ['rclc_parameter_server_init_with_option', 'rmw_uros_sync_session', 'rmw_uros_set_custom_transport']
        self.nm_output = '\n'.join('00000000 T ' + s for s in syms) + '\nTag_CPU_arch: v8-M.mainline\n'
        self.mock = patch.object(contract.subprocess, 'run', return_value=SimpleNamespace(stdout=self.nm_output))
        self.mock.start()
        self.addCleanup(self.mock.stop)

    def write(self, name, text):
        p = self.root / 'include' / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)

    def test_verified_pair_passes(self):
        self.assertEqual(contract.check(self.root, 'nm', record=True), [])
        self.assertEqual(contract.check(self.root, 'nm'), [])

    def test_old_mtu_rejected(self):
        self.write(contract.HEADERS[1], '#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU 512\n')
        self.assertTrue(any('MTU' in e for e in contract.check(self.root, 'nm', True)))

    def test_old_entity_limits_rejected(self):
        p = self.root / 'include' / contract.HEADERS[0]
        p.write_text(p.read_text().replace('PUBLISHERS 16', 'PUBLISHERS 12'))
        self.assertTrue(any('PUBLISHERS' in e for e in contract.check(self.root, 'nm', True)))

    def test_missing_symbol_rejected(self):
        with patch.object(contract.subprocess, 'run', return_value=SimpleNamespace(stdout='')):
            errors = contract.check(self.root, 'nm', True)
        self.assertTrue(any('Odometry' in e for e in errors))
        self.assertFalse((self.root / 'contract-sha256.json').exists())

    def test_archive_replacement_detected(self):
        contract.check(self.root, 'nm', True)
        (self.root / 'libmicroros.a').write_bytes(b'other archive')
        self.assertTrue(any('differ' in e for e in contract.check(self.root, 'nm')))

    def test_missing_manifest_rejected(self):
        self.assertTrue(any('manifest' in e for e in contract.check(self.root, 'nm')))

    def test_wrong_parameter_api_rejected(self):
        self.write(contract.HEADERS[2], '#define RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES 5\n')
        self.assertTrue(any('six' in e for e in contract.check(self.root, 'nm', True)))

    def test_hard_float_abi_rejected(self):
        with patch.object(contract.subprocess, 'run', return_value=SimpleNamespace(
                stdout=self.nm_output + 'Tag_ABI_VFP_args: VFP registers\n')):
            self.assertTrue(any('ABI' in e for e in contract.check(self.root, 'nm', True)))

    def test_wrong_cpu_abi_rejected(self):
        with patch.object(contract.subprocess, 'run', return_value=SimpleNamespace(
                stdout=self.nm_output.replace('v8-M.mainline', 'v6-M'))):
            self.assertTrue(any('ABI' in e for e in contract.check(self.root, 'nm', True)))

    def test_missing_header_or_archive_rejected(self):
        p = self.root / 'libmicroros.a'
        p.unlink()
        self.assertTrue(contract.check(self.root, 'nm', True))
        (self.root / 'include' / contract.HEADERS[0]).unlink()
        self.assertTrue(contract.check_headers(self.root))

if __name__ == '__main__':
    unittest.main()
