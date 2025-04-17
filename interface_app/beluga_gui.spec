# -*- mode: python -*-

import os
import sys
import time
import platform
from beluga_gui import __version__

app_name = f"Beluga_GUI_v{__version__}"

if platform.system() == 'Windows':
    app_name += '.exe'
    sys.path.append("c:\Windows\System32\downlevel")

a = Analysis(['main.py'],
             pathex=[os.getcwd()],
             hiddenimports=[],
             hookspath=None,
             datas=[],
             runtime_hooks=None)

pyz = PYZ(a.pure)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          name=app_name,
          debug=False,
          strip=None,
          upx=True,
          console=False,
          icon=None)

print("\nSuccessfully built at: %s" % time.strftime("%d %b %Y %I:%M:%S %p", time.localtime()))
