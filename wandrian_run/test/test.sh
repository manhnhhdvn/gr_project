rm -rf tmp
mkdir tmp
cp -r ../include ./tmp
cp -r ../src ./tmp

cd tmp/src/common
g++ -c point.cpp
g++ -c segment.cpp
g++ -c vector.cpp
g++ -c polygon.cpp
g++ -c environment.cpp

cd ../plans
g++ -c base_plan.cpp

cd wavefront
g++ -c cell.cpp
g++ -c wavefront.cpp

cd ../..
g++ -o ../wandrian_run ../../test.cpp \
common/point.o common/vector.o common/segment.o common/polygon.o common/environment.o \
plans/base_plan.o plans/wavefront/cell.o plans/wavefront/wavefront.o -lglut -lGL
cd ..
clear
clear
./wandrian_run $1

cd ..
rm -rf tmp