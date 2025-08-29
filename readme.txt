###########################################
1.Complete System Dependencies Installation
###########################################
sudo apt-get update
sudo apt-get install -y \
    git \
    cmake \
    build-essential \
    pkg-config \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libglu1-mesa-dev \
    xorg-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-dev \
    freeglut3-dev \
    libboost-all-dev \
    libeigen3-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libwayland-dev \
    libwayland-client0 \
    libwayland-cursor0 \
    libwayland-egl1 \
    g++ \
    clang \
    libc++-dev \
    libc++abi-dev
sudo apt-get clean
###########################################
2. install open3d
###########################################
cd ~/Workspace/Library
git clone https://github.com/isl-org/Open3D.git open3d
cd open3d
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON \
      -DBUILD_GUI=ON \
      -DBUILD_PYTHON_MODULE=OFF \
      -DBUILD_TENSORFLOW_OPS=OFF \
      -DBUILD_PYTORCH_OPS=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      -DBUILD_EXAMPLES=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=~/Workspace/Library/open3d_install \
      ..
make -j$(nproc)
sudo make install
###########################################
3. install tbb
###########################################
cd ~/Workspace/Library
git clone https://github.com/oneapi-src/oneTBB.git
cd oneTBB
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/Workspace/Library/tbb_install ..
make -j$(nproc)
sudo make install
###########################################
3. install gtsam
###########################################
cd ~/Workspace/Library
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.3.0
mkdir build && cd build
cmake .. \
    -DCMAKE_INSTALL_PREFIX=~/Workspace/Library/gtsam_install \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_TBB=ON \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DTBB_DIR=~/Workspace/Library/tbb_install/lib/cmake/TBB \
    -DCMAKE_CXX_FLAGS="-Wno-error=nonnull -Wno-error=stringop-overflow -Wno-error=maybe-uninitialized"
make -j$(nproc)
make check
sudo make install
###########################################
3. gtsam problem
###########################################
sudo nano /etc/ld.so.conf.d/gtsam-tbb.conf
/home/aswarm/Workspace/Library/gtsam_install/lib
/home/aswarm/Workspace/Library/tbb_install/lib
sudo ldconfig
####### verify >>
sudo ldconfig -p | grep libtbbmalloc
cd /home/aswarm/Workspace/Library/test/test_gtsam/build
./test_gtsam
###########################################
cd ~/Workspace/Library
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/Workspace/Library/sophus_install -DUSE_BASIC_LOGGING=ON -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
make check
sudo make install
