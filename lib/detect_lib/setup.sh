# TODO: apt packages

mkdir -p tensorflow
cd tensorflow

# git_sha=b0fcce8080f94b44195c429929361a420e9255cc

git_sha=733c2b04263e508fd08231fafaa745550687e4dd

git init
git remote add origin https://github.com/tensorflow/tensorflow.git
git fetch --depth=1 origin $git_sha
git checkout $git_sha

# rm -rf .git

cd ..
mkdir -p build
