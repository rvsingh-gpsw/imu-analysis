from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import os

has_cpp = os.path.isfile('imuanalysis.cpp')

extensions = [
    Extension(
      "imuanalysis", ["imuanalysis.pyx" if not has_cpp else "imuanalysis.cpp"],
    #   libraries=['GoProLibImganalysis'],
      libraries=['IMUAnalysis', 'gpmf_parser'],
      library_dirs=["../..", "../../gpmf"],
      include_dirs=["../../include", "../.."],
      extra_compile_args=[],
      language='c++')
]

setup(
  name = 'imu cython',
  ext_modules = cythonize(extensions, include_path=['../../include', "../.."]) if not has_cpp else extensions#, gdb_debug=True)
)
