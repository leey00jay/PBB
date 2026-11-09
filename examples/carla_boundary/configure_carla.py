#!/usr/bin/env python3
"""Back up and configure this CARLA installation for the laptop."""
from pathlib import Path
import os
import shutil
import time

carla_root = Path(os.environ.get('CARLA_ROOT', str(Path.home() / 'CARLA'))).expanduser()
path = carla_root / 'CarlaUE4/Config/DefaultEngine.ini'
text = path.read_text()
updated = text.replace('/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt',
                       '/Game/Carla/Maps/Town02_Opt.Town02_Opt')
updated = updated.replace('bSmoothFrameRate=false', 'bSmoothFrameRate=true')
updated = updated.replace(
    'SmoothedFrameRateRange=(LowerBound=(Type="ERangeBoundTypes::Inclusive",Value=22),UpperBound=(Type="ERangeBoundTypes::Exclusive",Value=120))',
    'SmoothedFrameRateRange=(LowerBound=(Type="ERangeBoundTypes::Inclusive",Value=10),UpperBound=(Type="ERangeBoundTypes::Exclusive",Value=20))')
if '[SystemSettings]' not in updated:
    updated += '\n[SystemSettings]\nt.MaxFPS=20\n'
if updated != text:
    backup = path.with_name(f'DefaultEngine.ini.before-carla-fix-{time.strftime("%Y%m%d-%H%M%S")}')
    shutil.copy2(path, backup)
    path.write_text(updated)
    print('Backup:', backup)
    print('Configured Town02_Opt and 20 FPS:', path)
else:
    print('Configuration already applied.')
