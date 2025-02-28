#!/usr/bin/env python
# mypy: ignore-errors

import platform
import os
import sys
import rospkg
import eel
import rospy
import argparse

# Actionsをインポートして、このファイルにバンドルする
from eel_ros1.actions import *  # noqa: F403
from eel_ros1.actions_util import *  # noqa: F403
from eel_ros1.models import rosparam # FIXME: おそらくros_serviceのインポートはここ必須

PACKAGE_NAME = "ply-measure-demo"
OPTIONS = {
    # "mode": "chrome-app",
    "host": "0.0.0.0",
    "port": 8000,
    'cmdline_args': ["--no-sandbox"],
    'size': (800, 600),
    "block": True
}

DIRECTORY = "build"
PAGE = "index.html"

rospack = rospkg.RosPack()
package_path = rospack.get_path(PACKAGE_NAME)
abs_path = os.path.join(package_path, 'web')

# コマンドライン引数の処理
argv = rospy.myargv(argv=sys.argv)
parser = argparse.ArgumentParser(description="EEL with React")
parser.add_argument("--html_dir", help="HTML directory path")
parser.add_argument("--port", help="Port number")
parser.add_argument("--dev", default=False, action="store_true", help="Development mode")
args = parser.parse_args(argv[1:])
if args.html_dir:
    abs_path = args.html_dir
if args.port:
    OPTIONS['port'] = args.port
if args.dev:
    DIRECTORY = "src"
    OPTIONS["mode"] = None
    PAGE = { "port": 3000 }

##### Mainブロック
rospy.init_node(PACKAGE_NAME, anonymous = True)
dist_path = os.path.join(abs_path, DIRECTORY)
print("Starting Eel app with React...")
print("  dist path: ", dist_path)
print("  hosted at:", f"http://{OPTIONS['host']}:{OPTIONS['port']}")
eel.init(dist_path, [".tsx", ".ts", ".jsx", ".js", ".html"])

eel.spawn(rosparam.getparam_loop)

try:
    eel.start(PAGE, **OPTIONS)
except EnvironmentError:
    if sys.platform in ["win32", "win64"] and int(platform.release()) >= 10:
        OPTIONS["mode"] = "edge"
        eel.start(PAGE, **OPTIONS)
    else:
        raise
