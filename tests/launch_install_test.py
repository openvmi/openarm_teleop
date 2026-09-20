"""Exercise launch scripts from a relocated install tree without opening CAN."""
import os
import itertools
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class LaunchInstallTest(unittest.TestCase):
    def test_relocated_install(self):
        source = Path(__file__).resolve().parents[1]
        for mode, explicit_ws in itertools.product(('unilateral', 'bilateral'), (False, True)):
            with self.subTest(mode=mode, explicit_ws=explicit_ws), tempfile.TemporaryDirectory(prefix='teleop relocated ') as temp:
                root = Path(temp)
                scripts = root / 'script'
                scripts.mkdir()
                shutil.copy(source / 'script' / f'launch_{mode}.sh', scripts)
                install = root / 'install'
                share = install / 'share' / 'openarm_teleop'
                (share / 'config').mkdir(parents=True)
                (share / 'config' / 'leader.yaml').touch()
                (share / 'config' / 'follower.yaml').touch()
                xacro = install / 'share/openarm_description/urdf/robot/oy.urdf.xacro'
                xacro.parent.mkdir(parents=True)
                xacro.touch()
                bin_dir = root / 'fake_bin'
                bin_dir.mkdir()
                (install / 'local_setup.bash').write_text('export PATH="$TEST_ROOT/fake_bin:$PATH"\n')
                programs = {
                    bin_dir / 'ros2': '''#!/bin/bash
if [[ "$*" == 'pkg prefix --share openarm_description' ]]; then
  echo "$TEST_ROOT/install/share/openarm_description"
elif [[ "$*" == 'pkg prefix --share openarm_teleop' ]]; then
  echo "$TEST_ROOT/install/share/openarm_teleop"
elif [[ "$*" == 'pkg prefix openarm_teleop' ]]; then
  echo "$TEST_ROOT/install"
else
  exit 1
fi
''',
                    bin_dir / 'pgrep': '#!/bin/bash\nexit 1\n',
                    bin_dir / 'xacro': '#!/bin/bash\n[[ "$1" == "$TEST_ROOT/install/share/openarm_description/urdf/robot/oy.urdf.xacro" ]] || exit 1\ntouch "$4"\n',
                    install / 'lib/openarm_teleop' / f'{mode}_control': '''#!/bin/bash
[[ "$PWD" == "$TEST_ROOT/install/share/openarm_teleop" ]] || exit 1
[[ -f config/leader.yaml && -f config/follower.yaml ]] || exit 1
[[ "$3 $4 $5 $6 $7" == 'right_arm can2 can0 0.8 0.6' ]] || exit 1
echo INSTALL_LAUNCH_OK
''',
                }
                for path, content in programs.items():
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(content)
                    path.chmod(0o755)
                env = dict(os.environ, TEST_ROOT=temp)
                env.pop('OPENARM_WS', None)
                if explicit_ws:
                    env['OPENARM_WS'] = temp
                result = subprocess.run(['bash', str(scripts / f'launch_{mode}.sh'), 'right_arm', '', '', '0.8', '0.6'], cwd='/', env=env, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('INSTALL_LAUNCH_OK', result.stdout)


if __name__ == '__main__':
    unittest.main()
