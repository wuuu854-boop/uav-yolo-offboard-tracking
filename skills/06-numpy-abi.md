# Skill 06 — Survive the numpy 2 / cv_bridge clash

| | |
|---|---|
| **When** | `cv_bridge` import or first use crashes with `_ARRAY_API not found` |
| **Time** | 5 minutes |
| **Needs** | A ROS 2 Humble environment |
| **From** | `ROS build` run #4 — exit code 139 |

## Symptom

```
AttributeError: _ARRAY_API not found
Fatal Python error: Segmentation fault
```

Often during `pytest`, or the first time a node converts an `Image`. The node may
run for a few seconds before dying.

## Why

ROS 2 Humble's `cv_bridge` and the system OpenCV are compiled against the
**numpy 1.x** C ABI. If `pip install` pulled numpy 2.x into a directory that comes
earlier on `sys.path` than the apt-installed numpy, the extension module loads
against the wrong ABI and the process segfaults.

It is not a Python error you can catch: the interpreter dies.

## Procedure

1. See both numpys:

   ```bash
   python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
   dpkg -l | grep -E "python3-numpy|python3-opencv"
   ```

   A `2.x` version installed under `/usr/local/lib/python3.10/dist-packages`
   alongside apt's `1.26` is the tell.

2. Pin the runtime to numpy 1.x:

   ```bash
   python3 -m pip install "numpy<2"
   ```

   In this project it is pinned in `requirements.txt`, with the reason next to it.

3. Do not "fix" it by upgrading OpenCV. The problem is the ABI of the ROS
   packages, and they will not follow.

## Verify

```bash
python3 -c "import numpy; print(numpy.__version__)"
python3 -c "from cv_bridge import CvBridge; CvBridge(); print('cv_bridge ok')"
```

Both must succeed, and the version must start with `1.`. Then run the test that
used to crash.

## Generalisation

Any time a compiled ROS package and a pip package share a C dependency, the pip
one wins if it lands earlier on the path — and the failure is a segfault, not an
exception. Pin the shared dependency where the ROS distribution expects it.
