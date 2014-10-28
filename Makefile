OMPL_DIR = /usr
#CXXFLAGS = -O2 # change to -g when debugging code
CXXFLAGS = -g -std=c++11
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options -lboost_system
CXX=c++

Controls: ODESolverAndControlsWithRigidBodyPlanning.o RGRRT.o
	$(CXX) $(CXXFLAGS) -o Controls ODESolverAndControlsWithRigidBodyPlanning.o RGRRT.o $(LD_FLAGS)

clean:
	rm -rf *.o
	rm -rf *.db
	rm -rf path.txt
	rm -rf Controls

plots:
	ompl_benchmark_statistics.py Benchmark-Pendulum-15.log -d Pen-15.db -p Pen-15.pdf
	ompl_benchmark_statistics.py Benchmark-KinodynamicCar-15.log -d Car-15.db -p Car-15.pdf

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
