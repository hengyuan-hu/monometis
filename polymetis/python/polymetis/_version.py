import os
import json

import polymetis

__version__ = ""

# Conda installed: Get version of conda pkg (assigned $GIT_DESCRIBE_NUMBER during build)
if "CONDA_PREFIX" in os.environ and os.environ["CONDA_PREFIX"] in polymetis.__file__:
    # Search conda pkgs for polymetis & extract version number
    stream = os.popen("conda list | grep polymetis")
    for line in stream:
        info_fields = [s for s in line.strip("\n").split(" ") if len(s) > 0]
        if info_fields[0] == "polymetis":  # pkg name == polymetis
            __version__ = info_fields[1]
            break

# Built locally: Retrive git tag description of Polymetis source code
else:
    # Navigate to polymetis pkg dir, which should be within the git repo
    original_cwd = os.getcwd()
    os.chdir(os.path.dirname(polymetis.__file__))

    commit = [f for f in os.popen("git log -1")][0]
    __version__ = commit.strip("\n").split()[-1]
    print(f"version: (git) {__version__}")

    # Reset cwd
    os.chdir(original_cwd)

if not __version__:
    raise Exception("Cannot locate Polymetis version!")
