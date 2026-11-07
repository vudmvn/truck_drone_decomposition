cd /home/orlab/Minh_ALL/truck_drone_decomposition

rm -rf build
mkdir build
cd build

cmake .. \
  -DGUROBI_HOME=/home/orlab/gurobi1301/linux64 \
  -DORTOOLS_HOME=/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963 \
  -DCMAKE_PREFIX_PATH=/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963

cmake --build . -j