OMPL_DIR = /usr
#CXXFLAGS = -O2 # change to -g when debugging code
CXXFLAGS = -g -std=c++11
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options -lboost_system
CXX=c++

Controls: RigidBodyPlanningWithODESolverAndControls.o
	$(CXX) $(CXXFLAGS) -o Controls RigidBodyPlanningWithODESolverAndControls.o $(LD_FLAGS)

clean:
	rm *.o

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
