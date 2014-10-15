OMPL_DIR = /usr
CXXFLAGS = -O2 # change to -g when debugging code
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options
CXX=c++

myprogram: myprogram.o myclass.o
	$(CXX) $(CXXFLAGS) -o myprogram myprogram.o myclass.o $(LD_FLAGS)

clean:
	rm *.o

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
