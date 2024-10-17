import urllib.request
import zipfile
import os
import shutil
import sys

try:
    branch = 'main'

    if len(sys.argv) > 1:
        branch = sys.argv[1]

    temp = 'temp'
    os.makedirs(temp, exist_ok=True)
    path = os.path.join(temp, 'source.zip')
    urllib.request.urlretrieve(f'https://github.com/RandomPythonProgrammer/Library/archive/refs/heads/{branch}.zip', path)
    with zipfile.ZipFile(path, 'r') as file:
        file.extractall(temp)

    os.makedirs('include', exist_ok=True)
    os.makedirs('src', exist_ok=True)

    extracted = os.path.join(temp, f'Library-{branch}')

    shutil.copytree(os.path.join(extracted, 'src'), 'src', dirs_exist_ok=True)
    shutil.copytree(os.path.join(extracted, 'include'), 'include', dirs_exist_ok=True)
finally:
    shutil.rmtree(temp)