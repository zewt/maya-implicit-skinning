
if [ $# -ne 1 ]
then                    
  echo "Usage: first argument must be the compute capability of the GPU (sm_10 sm_13 sm_XX etc.)"
  exit 1
fi

cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DCUDA_COMPUTE_CAPABILITY=$1 ./..
cd ..
